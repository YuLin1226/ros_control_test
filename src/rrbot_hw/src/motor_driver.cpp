#include "../include/rrbot_hw/motor_driver.h"
#include <cmath>


namespace Motor{

    union unionType
    {
        uint8_t _data_byte[2];
        uint16_t _data;
    };


    MotorDriver::MotorDriver(const std::string _serial_port = "/dev/ttyUSB0", const int _baud_rate = 115200) : SerialModbus(_serial_port, _baud_rate){
        p_func_mutex = std::shared_ptr<boost::mutex>{new boost::mutex};
    }
    MotorDriver::~MotorDriver(){
        // STOP Motor
        this->ISTOP(false);
        // this->FREE(false);
    }



    void MotorDriver::JG(uint16_t _cmd_rpm, bool is_echo = false){

        /* ==============================================================================
            *     0 < _cmd_rpm < 4000  :  CW
            * -4000 < _cmd_rpm <    0  : CCW
            * 
            * Example: Set +300 rpm / No Echo
            * 0 65 1 1 6e 0 0 1 2c 34 72
            * 
            * If echo, receive 8 bytes data per message.
        ============================================================================== */

        std::vector<uint8_t> p_data;
        uint8_t _num = 0x01;
        uint8_t _cmd = is_echo ? this->CMD_JG_Echo : this->CMD_JG_No_Echo;
        unionType _data_1, _data_2;
        _data_1._data = 0x00;
        _data_2._data = _cmd_rpm;

        p_data.clear();
        p_data.push_back(this->Broadcast);
        p_data.push_back(this->FC_MasterSendCMD);
        p_data.push_back(_num);
        p_data.push_back(this->MOTOR_ID);
        p_data.push_back(_cmd);
        p_data.push_back(_data_1._data_byte[1]);
        p_data.push_back(_data_1._data_byte[0]);
        p_data.push_back(_data_2._data_byte[1]);
        p_data.push_back(_data_2._data_byte[0]);
        uint16_t crc = this->calculate_CRC(p_data);
        p_data.push_back(crc);
        p_data.push_back(crc >> 8);
        std::vector<char> p_char(p_data.begin(), p_data.end());
        this->write(p_char);

        std::cout <<  "Send JG Command: ";
        for(auto i=0;i<p_char.size();i++){            
            std::cout << std::hex << (int)p_data[i] << " ";
        }
        std::cout << std::endl;
    }

    void MotorDriver::ISTOP(bool is_echo = false){

        /* ==============================================================================
            * Example: No Echo
            * 0 65 1 1 64 0 0 0 0 ac 3e
            * 
            * If echo, receive 8 bytes data per message.
        ============================================================================== */
        
        std::vector<uint8_t> p_data;
        uint8_t _num = 0x01;
        uint8_t _cmd = is_echo ? this->CMD_ISTOP_Echo : this->CMD_ISTOP_No_Echo;
        unionType _data_1, _data_2;
        _data_1._data = 0x00;
        _data_2._data = 0x00;

        p_data.clear();
        p_data.push_back(this->Broadcast);
        p_data.push_back(this->FC_MasterSendCMD);
        p_data.push_back(_num);
        p_data.push_back(this->MOTOR_ID);
        p_data.push_back(_cmd);
        p_data.push_back(_data_1._data_byte[1]);
        p_data.push_back(_data_1._data_byte[0]);
        p_data.push_back(_data_2._data_byte[1]);
        p_data.push_back(_data_2._data_byte[0]);
        uint16_t crc = this->calculate_CRC(p_data);
        p_data.push_back(crc);
        p_data.push_back(crc >> 8);
        std::vector<char> p_char(p_data.begin(), p_data.end());
        this->write(p_char);

        std::cout <<  "Send ISTOP Command: ";
        for(auto i=0;i<p_char.size();i++){            
            std::cout << std::hex << (int)p_data[i] << " ";
        }
        std::cout << std::endl;
    }

    void MotorDriver::FREE(bool is_echo = false){

        /* ==============================================================================
            * Example: No Echo
            * 0 65 1 1 69 0 0 0 0 81 ff
            * 
            * If echo, receive 8 bytes data per message.
        ============================================================================== */
        
        std::vector<uint8_t> p_data;
        uint8_t _num = 0x01;
        uint8_t _cmd = is_echo ? this->CMD_FREE_Echo : this->CMD_FREE_No_Echo;
        unionType _data_1, _data_2;
        _data_1._data = 0x00;
        _data_2._data = 0x00;

        p_data.clear();
        p_data.push_back(this->Broadcast);
        p_data.push_back(this->FC_MasterSendCMD);
        p_data.push_back(_num);
        p_data.push_back(this->MOTOR_ID);
        p_data.push_back(_cmd);
        p_data.push_back(_data_1._data_byte[1]);
        p_data.push_back(_data_1._data_byte[0]);
        p_data.push_back(_data_2._data_byte[1]);
        p_data.push_back(_data_2._data_byte[0]);
        uint16_t crc = this->calculate_CRC(p_data);
        p_data.push_back(crc);
        p_data.push_back(crc >> 8);
        std::vector<char> p_char(p_data.begin(), p_data.end());
        this->write(p_char);

        std::cout <<  "Send FREE Command: ";
        for(auto i=0;i<p_char.size();i++){            
            std::cout << std::hex << (int)p_data[i] << " ";
        }
        std::cout << std::endl;
    }

    void MotorDriver::SVON(bool is_echo = false){

        /* ==============================================================================
            * If echo, receive 8 bytes data per message.
        ============================================================================== */

        std::vector<uint8_t> p_data;
        uint8_t _num = 0x01;
        uint8_t _cmd = is_echo ? this->CMD_SVON_Echo : this->CMD_SVON_No_Echo;
        unionType _data_1, _data_2;
        _data_1._data = 0x00;
        _data_2._data = 0x00;

        p_data.clear();
        p_data.push_back(this->Broadcast);
        p_data.push_back(this->FC_MasterSendCMD);
        p_data.push_back(_num);
        p_data.push_back(this->MOTOR_ID);
        p_data.push_back(_cmd);
        p_data.push_back(_data_1._data_byte[1]);
        p_data.push_back(_data_1._data_byte[0]);
        p_data.push_back(_data_2._data_byte[1]);
        p_data.push_back(_data_2._data_byte[0]);
        uint16_t crc = this->calculate_CRC(p_data);
        p_data.push_back(crc);
        p_data.push_back(crc >> 8);
        std::vector<char> p_char(p_data.begin(), p_data.end());
        this->write(p_char);

        std::cout <<  "Send SVON Command: ";
        for(auto i=0;i<p_char.size();i++){            
            std::cout << std::hex << (int)p_data[i] << " ";
        }
        std::cout << std::endl;

    }

    void MotorDriver::IMR(uint16_t _index, uint16_t _step, bool is_echo = false){
        
        /* ==============================================================================
            * If echo, receive 8 bytes data per message.
        ============================================================================== */

        std::vector<uint8_t> p_data;
        uint8_t _num = 0x01;
        uint8_t _cmd = is_echo ? this->CMD_IMR_Echo : this->CMD_IMR_No_Echo;
        unionType _data_1, _data_2;
        _data_1._data = _index;
        _data_2._data = _step;

        p_data.clear();
        p_data.push_back(this->Broadcast);
        p_data.push_back(this->FC_MasterSendCMD);
        p_data.push_back(_num);
        p_data.push_back(this->MOTOR_ID);
        p_data.push_back(_cmd);
        p_data.push_back(_data_1._data_byte[1]);
        p_data.push_back(_data_1._data_byte[0]);
        p_data.push_back(_data_2._data_byte[1]);
        p_data.push_back(_data_2._data_byte[0]);
        uint16_t crc = this->calculate_CRC(p_data);
        p_data.push_back(crc);
        p_data.push_back(crc >> 8);
        std::vector<char> p_char(p_data.begin(), p_data.end());
        this->write(p_char);

        std::cout <<  "Send IMR Command: ";
        for(auto i=0;i<p_char.size();i++){            
            std::cout << std::hex << (int)p_data[i] << " ";
        }
        std::cout << std::endl;

    }

    void MotorDriver::CS(uint16_t _index, uint16_t _step, bool is_echo = false){
        
        /* ==============================================================================
            * If echo, receive 8 bytes data per message.
        ============================================================================== */

        std::vector<uint8_t> p_data;
        uint8_t _num = 0x01;
        uint8_t _cmd = is_echo ? this->CMD_CS_Echo : this->CMD_CS_No_Echo;
        unionType _data_1, _data_2;
        _data_1._data = _index;
        _data_2._data = _step;

        p_data.clear();
        p_data.push_back(this->Broadcast);
        p_data.push_back(this->FC_MasterSendCMD);
        p_data.push_back(_num);
        p_data.push_back(this->MOTOR_ID);
        p_data.push_back(_cmd);
        p_data.push_back(_data_1._data_byte[1]);
        p_data.push_back(_data_1._data_byte[0]);
        p_data.push_back(_data_2._data_byte[1]);
        p_data.push_back(_data_2._data_byte[0]);
        uint16_t crc = this->calculate_CRC(p_data);
        p_data.push_back(crc);
        p_data.push_back(crc >> 8);
        std::vector<char> p_char(p_data.begin(), p_data.end());
        this->write(p_char);

        std::cout <<  "Send CS Command: ";
        for(auto i=0;i<p_char.size();i++){            
            std::cout << std::hex << (int)p_data[i] << " ";
        }
        std::cout << std::endl;

    }

    void MotorDriver::CMR(uint16_t _index, uint16_t _step, bool is_echo = false){
        
        /* ==============================================================================
            * If echo, receive 8 bytes data per message.
        ============================================================================== */

        std::vector<uint8_t> p_data;
        uint8_t _num = 0x01;
        uint8_t _cmd = is_echo ? this->CMD_CMR_Echo : this->CMD_CMR_No_Echo;
        unionType _data_1, _data_2;
        _data_1._data = _index;
        _data_2._data = _step;

        p_data.clear();
        p_data.push_back(this->Broadcast);
        p_data.push_back(this->FC_MasterSendCMD);
        p_data.push_back(_num);
        p_data.push_back(this->MOTOR_ID);
        p_data.push_back(_cmd);
        p_data.push_back(_data_1._data_byte[1]);
        p_data.push_back(_data_1._data_byte[0]);
        p_data.push_back(_data_2._data_byte[1]);
        p_data.push_back(_data_2._data_byte[0]);
        uint16_t crc = this->calculate_CRC(p_data);
        p_data.push_back(crc);
        p_data.push_back(crc >> 8);
        std::vector<char> p_char(p_data.begin(), p_data.end());
        this->write(p_char);

        std::cout <<  "Send CMR Command: ";
        for(auto i=0;i<p_char.size();i++){            
            std::cout << std::hex << (int)p_data[i] << " ";
        }
        std::cout << std::endl;

    }

    void MotorDriver::CMA(uint16_t _index, uint16_t _step, bool is_echo = false){
        
        /* ==============================================================================
            * If echo, receive 8 bytes data per message.
        ============================================================================== */

        std::vector<uint8_t> p_data;
        uint8_t _num = 0x01;
        uint8_t _cmd = is_echo ? this->CMD_CMA_Echo : this->CMD_CMA_No_Echo;
        unionType _data_1, _data_2;
        _data_1._data = _index;
        _data_2._data = _step;

        p_data.clear();
        p_data.push_back(this->Broadcast);
        p_data.push_back(this->FC_MasterSendCMD);
        p_data.push_back(_num);
        p_data.push_back(this->MOTOR_ID);
        p_data.push_back(_cmd);
        p_data.push_back(_data_1._data_byte[1]);
        p_data.push_back(_data_1._data_byte[0]);
        p_data.push_back(_data_2._data_byte[1]);
        p_data.push_back(_data_2._data_byte[0]);
        uint16_t crc = this->calculate_CRC(p_data);
        p_data.push_back(crc);
        p_data.push_back(crc >> 8);
        std::vector<char> p_char(p_data.begin(), p_data.end());
        this->write(p_char);

        std::cout <<  "Send CMA Command: ";
        for(auto i=0;i<p_char.size();i++){            
            std::cout << std::hex << (int)p_data[i] << " ";
        }
        std::cout << std::endl;

    }

    void MotorDriver::NULL_TO_ECHO(bool is_echo = true){
        
        /* ==============================================================================
            * If echo, receive 8 bytes data per message.
        ============================================================================== */

        std::vector<uint8_t> p_data;
        uint8_t _num = 0x01;
        uint8_t _cmd = is_echo ? this->CMD_NULL_Echo : this->CMD_NULL_No_Echo;
        unionType _data_1, _data_2;
        _data_1._data = 0x00;
        _data_2._data = 0x00;

        p_data.clear();
        p_data.push_back(this->Broadcast);
        p_data.push_back(this->FC_MasterSendCMD);
        p_data.push_back(_num);
        p_data.push_back(this->MOTOR_ID);
        p_data.push_back(_cmd);
        p_data.push_back(_data_1._data_byte[1]);
        p_data.push_back(_data_1._data_byte[0]);
        p_data.push_back(_data_2._data_byte[1]);
        p_data.push_back(_data_2._data_byte[0]);
        uint16_t crc = this->calculate_CRC(p_data);
        p_data.push_back(crc);
        p_data.push_back(crc >> 8);
        std::vector<char> p_char(p_data.begin(), p_data.end());
        this->write(p_char);

        std::cout <<  "Send NULL Command: ";
        for(auto i=0;i<p_char.size();i++){            
            std::cout << std::hex << (int)p_data[i] << " ";
        }
        std::cout << std::endl;

        if(is_echo){

            std::vector<char> rcv_char;
            
            {
                usleep(RESPONSE_DELAY_US);
                try
                {
                    rcv_char = asyncRead(this->rcv_size);
                }
                catch(const std::exception& e)
                {
                    std::cerr << e.what() << '\n';
                }
                std::cout <<  "Received Data: ";
                for(auto i=0;i<rcv_char.size();i++){            
                    uint8_t a = rcv_char[i];
                    std::cout << std::hex << +a << " ";
                }
                std::cout << std::endl;
                
            }

        }
    }

    double MotorDriver::get_Encoder(){
        const int expected_bytes = 8;

        this->NULL_TO_ECHO(true);        
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

        double encoder_data_;
        union unionType encoder_index_, encoder_step_;
        encoder_index_._data_byte[1]    = response.at(2);
        encoder_index_._data_byte[0]    = response.at(3);
        encoder_step_._data_byte[1]     = response.at(4);
        encoder_step_._data_byte[0]     = response.at(5);
        encoder_data_ = (encoder_index_._data + encoder_step_._data/10000)*M_PI/360;

        return encoder_data_;
    }

}