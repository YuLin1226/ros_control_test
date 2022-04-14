#include <rrbot_hw/modbus.h>

#define WRITE_DELAY 10000

namespace Motor
{
    SerialModbus::SerialModbus( const std::string _serial_port = "/dev/ttyUSB0",
                                const int _baud_rate = 115200){
        set_SerialPort_Param(_serial_port, _baud_rate);
    }
    
    SerialModbus::~SerialModbus(){
        close();
    }

    void SerialModbus::set_SerialPort_Param(const std::string _serial_port, const int _baud_rate){
        p_serial_port   = _serial_port;
        p_baud_rate     = _baud_rate;
    }

    int SerialModbus::open(){
        if(p_port != NULL)
        {
            return -1;
        }
        if(p_service)
        {
            p_port.reset();
            p_service.reset();
        }
        p_mutex     = std::shared_ptr<boost::mutex>{new boost::mutex};
        p_service   = std::shared_ptr<io_service>{new io_service()};
        p_port      = std::shared_ptr<serial_port>{ new serial_port(*p_service) };
        p_timeout   = std::shared_ptr<deadline_timer>{new deadline_timer(*p_service)};

        try 
        {
            /* ======================= 設置Serial Port參數 ======================= */
            p_port->open(p_serial_port);
            p_port->set_option(serial_port_base::baud_rate(p_baud_rate));
            p_port->set_option(serial_port_base::character_size(8));
            p_port->set_option(serial_port_base::parity(serial_port_base::parity::none));
            p_port->set_option(serial_port_base::stop_bits(serial_port_base::stop_bits::one));
            std::cout << "Serial Connect." << std::endl;
        }
        catch (std::exception &ex){
            std::cout << "Open Exception : " << ex.what() << std::endl;
        }
        return 0;           
    }

    int SerialModbus::close(){
        if(p_port && p_port->is_open())
        {
            p_port->close();
        }
        std::cout << "Serial Disconnect." << std::endl;
        return 0;           
    }

    void SerialModbus::write(std::vector<char> _data){
	    if (p_port->is_open())
        {
            boost::mutex::scoped_lock lock(*p_mutex);
            auto size = p_port->write_some(buffer(_data));
            if(_data.size() != size)
            {
                throw "Write Size Error.";
            }      
        }
        else
        {
            throw "Serial Port Is Not Open.";
        }
    }    
    
    void SerialModbus::write_to_single_register(uint8_t _id, uint8_t _function_code, uint16_t _addr, uint16_t _data)
    {
        /* =====================================================================================
            * 建立Bytes資料容器
            * 存入 ID, FunctionCode, Address, Data, CRC
            * 轉換Char資料容器
            * 發送資料
           ===================================================================================== */
        std::vector<uint8_t> p_data;
        p_data.clear();
        p_data.push_back(_id);
        p_data.push_back(_function_code);
        p_data.push_back(_addr >> 8);
        p_data.push_back(_addr);
        p_data.push_back(_data >> 8);
        p_data.push_back(_data);
        uint16_t crc = this->calculate_CRC(p_data);
        p_data.push_back(crc >> 8);
        p_data.push_back(crc);
        std::vector<char> p_char(p_data.begin(), p_data.end());
        this->write(p_char);
    }

    std::vector<char> SerialModbus::asyncRead(size_t rcv_size)
    {
        p_service->reset();
        p_available = false;
        std::vector<char> p_char(rcv_size);
        try
        {
            /* =============================== 邏輯解釋 ===============================
                * scoped_lock 用於保護線程。
                * async_read 用於異步讀取資料，並且讀取到 "rcv_size" 長度的字節才會呼叫Lambda表達式，即讀取完畢。
                *   1) 讀取完畢的error_code  = 0 (False)
                * timeout expires_from_now 設定時間。
                * timeout async_wait 等待已設定的時間，其呼叫 Lambda 有兩種情況：
                *   1) 過期，此時error_code  = 0 (False)
                *   2) 取消，此時error_code != 0 (True)
                * service_run 阻塞後續工作，等待線程內的事項結束。
                ---------------------------------------------------------------------
                1-1) 讀取 -> 成功 -> 取消定時器 -> 定時器不做事。
                1-2) 讀取 -> 失敗 -> 拋出錯誤
                2-1) 定時器 -> 過期 -> 取消序列埠工作 -> 拋出錯誤              
               =============================== 邏輯解釋 ===============================*/
            boost::mutex::scoped_lock sl(*p_mutex);
            async_read( *p_port, 
                        boost::asio::buffer(p_char, rcv_size),
                        [&](const boost::system::error_code &error, std::size_t bytes_transferred)
                        {
                            if (error)
                            {
                                p_available = false;
                                std::cerr << "readCallback Error " << error << std::endl;
                            }
                            p_timeout->cancel();
                            p_available = true;
                        });
            p_timeout->expires_from_now(boost::posix_time::millisec(READ_TIME_OUT_MS));
            p_timeout->async_wait(  [&](const boost::system::error_code &error)
                                    {
                                        if (!error)
                                        {
                                            p_available = false;
                                            p_port->cancel(); 
                                            std::cerr << "Read timeout" << std::endl;
                                        }
                                    });
            p_service->run(); 
        }
        catch(const std::exception& ex)
        {
            std::cout << "Read exception. " << ex.what() << std::endl;
        }
        if (p_available)
        {
            return p_char;
        }
        else
        {
            throw "Serial port reading timeout";
        }
    }

    std::vector<char> SerialModbus::read_and_write(uint8_t _ID, uint8_t _FC, uint16_t _ADDR, uint16_t _DATA, int expected_bytes){
    
        /* 這個功能之後會作為特定函數的模板，例如：get_MotorSpeed，那就是先寫入再讀取 */
        write_to_single_register(_ID, _FC, _ADDR, _DATA);    
        std::vector<char> response;
        {
            usleep(RESPONSE_DELAY_US);
            try
            {
                response = asyncRead(expected_bytes);
            }
            catch(const std::exception& e)
            {
                std::cerr << e.what() << '\n';
            }
        }
        return response;
    }
}
