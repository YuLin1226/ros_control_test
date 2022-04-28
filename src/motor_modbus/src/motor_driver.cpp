#include <motor_modbus/motor_driver.h>
#include <cmath>

namespace Motor{

    union unionType
    {
        uint8_t _data_byte[2]; // [1] 高8位,上位 ； [0] 低8位,下位
        int16_t _data;
    };


    MotorDriver::MotorDriver(const std::string _serial_port = "/dev/ttyUSB0", const int _baud_rate = 115200) : SerialModbus(_serial_port, _baud_rate){
        p_func_mutex = std::shared_ptr<boost::mutex>{new boost::mutex};
    }
    MotorDriver::~MotorDriver(){
        // STOP Motor
        uint8_t num_ = 2;
        std::vector<uint8_t> drive_motor_id_({0x01, 0x02});
        this->Multi_ISTOP_Lite(num_, drive_motor_id_, false);

        sleep(1);
        std::vector<uint8_t> steer_motor_id_({0x03, 0x04});
        this->Multi_ISTOP(num_, steer_motor_id_, false);
        // this->ISTOP(false);
        // this->FREE(false);
    }


    // Motor Drive
    void MotorDriver::ISTOP(uint8_t id_ = 0x01, bool is_echo = false){

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
        p_data.push_back(id_);
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

    void MotorDriver::JG(uint8_t id_ = 0x01, int16_t _cmd_rpm = 0, bool is_echo = false){

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
        p_data.push_back(id_);
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

    void MotorDriver::FREE(uint8_t id_ = 0x01, bool is_echo = false){

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
        p_data.push_back(id_);
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

    void MotorDriver::SVON(uint8_t id_ = 0x01, bool is_echo = false){

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
        p_data.push_back(id_);
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

    void MotorDriver::IMR(uint8_t id_ = 0x01, int16_t _index = 0, uint16_t _step = 0, bool is_echo = false){
        
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
        p_data.push_back(id_);
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

    void MotorDriver::CS(uint8_t id_ = 0x01, int16_t _index = 0, uint16_t _step = 0, bool is_echo = false){
        
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
        p_data.push_back(id_);
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

    void MotorDriver::CMR(uint8_t id_ = 0x01, int16_t _index = 0, uint16_t _step = 0, bool is_echo = false){
        
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
        p_data.push_back(id_);
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

    void MotorDriver::CMA(uint8_t id_ = 0x01, int16_t _index = 0, uint16_t _step = 0, bool is_echo = false){
        
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
        p_data.push_back(id_);
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

    void MotorDriver::NULL_TO_ECHO(uint8_t id_ = 0x01, bool is_echo = true){
        
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
        p_data.push_back(id_);
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

        // if(is_echo){

        //     std::vector<char> rcv_char;
            
        //     {
        //         usleep(RESPONSE_DELAY_US);
        //         try
        //         {
        //             rcv_char = asyncRead(this->rcv_size);
        //         }
        //         catch(const std::exception& e)
        //         {
        //             std::cerr << e.what() << '\n';
        //         }
        //         std::cout <<  "Received Data: ";
        //         for(auto i=0;i<rcv_char.size();i++){            
        //             uint8_t a = rcv_char[i];
        //             std::cout << std::hex << +a << " ";
        //         }
        //         std::cout << std::endl;
                
        //     }

        // }
    }

    // Motor Drive Lite
    void MotorDriver::ISTOP_Lite(bool is_echo = false, uint8_t id_ = 0x01){

        std::vector<uint8_t> p_data;
        uint8_t _num = 0x01;
        uint8_t _echo_bit = is_echo ? this->Echo_bit_Lite : this->No_Echo_bit_Lite;
        unionType _data_1, _data_2;
        _data_1._data = 0x00;
        _data_2._data = _echo_bit;

        p_data.clear();
        p_data.push_back(this->Broadcast);
        p_data.push_back(this->FC_MasterSendCMD_Lite);
        p_data.push_back(_num);
        p_data.push_back(id_);
        p_data.push_back(this->CMD_ISTOP_Lite);
        p_data.push_back(_data_1._data_byte[1]);
        p_data.push_back(_data_1._data_byte[0]);
        p_data.push_back(_data_2._data_byte[1]);
        p_data.push_back(_data_2._data_byte[0]);
        uint16_t crc = this->calculate_CRC(p_data);
        p_data.push_back(crc);
        p_data.push_back(crc >> 8);
        std::vector<char> p_char(p_data.begin(), p_data.end());
        this->write(p_char);

        std::cout <<  "Send ISTOP_Lite Command: ";
        for(auto i=0;i<p_char.size();i++){            
            std::cout << std::hex << (int)p_data[i] << " ";
        }
        std::cout << std::endl;
    }

    void MotorDriver::JG_Lite(uint16_t _cmd_rpm, bool is_echo = false, uint8_t id_ = 0x01){

        std::vector<uint8_t> p_data;
        uint8_t _num = 0x01;
        uint8_t _echo_bit = is_echo ? this->Echo_bit_Lite : this->No_Echo_bit_Lite;
        unionType _data_1, _data_2;
        _data_1._data = _cmd_rpm;
        _data_2._data = _echo_bit;

        p_data.clear();
        p_data.push_back(this->Broadcast);
        p_data.push_back(this->FC_MasterSendCMD_Lite);
        p_data.push_back(_num);
        p_data.push_back(id_);
        p_data.push_back(this->CMD_JG_Lite);
        p_data.push_back(_data_1._data_byte[1]);
        p_data.push_back(_data_1._data_byte[0]);
        p_data.push_back(_data_2._data_byte[1]);
        p_data.push_back(_data_2._data_byte[0]);
        uint16_t crc = this->calculate_CRC(p_data);
        p_data.push_back(crc);
        p_data.push_back(crc >> 8);
        std::vector<char> p_char(p_data.begin(), p_data.end());
        this->write(p_char);

        std::cout <<  "Send JG_Lite Command: ";
        for(auto i=0;i<p_char.size();i++){            
            std::cout << std::hex << (int)p_data[i] << " ";
        }
        std::cout << std::endl;
    }

    void MotorDriver::FREE_Lite(bool is_echo = false, uint8_t id_ = 0x01){

        std::vector<uint8_t> p_data;
        uint8_t _num = 0x01;
        uint8_t _echo_bit = is_echo ? this->Echo_bit_Lite : this->No_Echo_bit_Lite;
        unionType _data_1, _data_2;
        _data_1._data = 0x00;
        _data_2._data = _echo_bit;

        p_data.clear();
        p_data.push_back(this->Broadcast);
        p_data.push_back(this->FC_MasterSendCMD_Lite);
        p_data.push_back(_num);
        p_data.push_back(id_);
        p_data.push_back(this->CMD_FREE_Lite);
        p_data.push_back(_data_1._data_byte[1]);
        p_data.push_back(_data_1._data_byte[0]);
        p_data.push_back(_data_2._data_byte[1]);
        p_data.push_back(_data_2._data_byte[0]);
        uint16_t crc = this->calculate_CRC(p_data);
        p_data.push_back(crc);
        p_data.push_back(crc >> 8);
        std::vector<char> p_char(p_data.begin(), p_data.end());
        this->write(p_char);

        std::cout <<  "Send FREE_Lite Command: ";
        for(auto i=0;i<p_char.size();i++){            
            std::cout << std::hex << (int)p_data[i] << " ";
        }
        std::cout << std::endl;
    }

    void MotorDriver::SVON_Lite(bool is_echo = false, uint8_t id_ = 0x01){

        std::vector<uint8_t> p_data;
        uint8_t _num = 0x01;
        uint8_t _echo_bit = is_echo ? this->Echo_bit_Lite : this->No_Echo_bit_Lite;
        unionType _data_1, _data_2;
        _data_1._data = 0x00;
        _data_2._data = _echo_bit;

        p_data.clear();
        p_data.push_back(this->Broadcast);
        p_data.push_back(this->FC_MasterSendCMD_Lite);
        p_data.push_back(_num);
        p_data.push_back(id_);
        p_data.push_back(this->CMD_SVON_Lite);
        p_data.push_back(_data_1._data_byte[1]);
        p_data.push_back(_data_1._data_byte[0]);
        p_data.push_back(_data_2._data_byte[1]);
        p_data.push_back(_data_2._data_byte[0]);
        uint16_t crc = this->calculate_CRC(p_data);
        p_data.push_back(crc);
        p_data.push_back(crc >> 8);
        std::vector<char> p_char(p_data.begin(), p_data.end());
        this->write(p_char);

        std::cout <<  "Send SVON_Lite Command: ";
        for(auto i=0;i<p_char.size();i++){            
            std::cout << std::hex << (int)p_data[i] << " ";
        }
        std::cout << std::endl;
    }

    void MotorDriver::SVOFF_Lite(bool is_echo = false, uint8_t id_ = 0x01){

        std::vector<uint8_t> p_data;
        uint8_t _num = 0x01;
        uint8_t _echo_bit = is_echo ? this->Echo_bit_Lite : this->No_Echo_bit_Lite;
        unionType _data_1, _data_2;
        _data_1._data = 0x00;
        _data_2._data = _echo_bit;

        p_data.clear();
        p_data.push_back(this->Broadcast);
        p_data.push_back(this->FC_MasterSendCMD_Lite);
        p_data.push_back(_num);
        p_data.push_back(id_);
        p_data.push_back(this->CMD_SVOFF_Lite);
        p_data.push_back(_data_1._data_byte[1]);
        p_data.push_back(_data_1._data_byte[0]);
        p_data.push_back(_data_2._data_byte[1]);
        p_data.push_back(_data_2._data_byte[0]);
        uint16_t crc = this->calculate_CRC(p_data);
        p_data.push_back(crc);
        p_data.push_back(crc >> 8);
        std::vector<char> p_char(p_data.begin(), p_data.end());
        this->write(p_char);

        std::cout <<  "Send SVOFF_Lite Command: ";
        for(auto i=0;i<p_char.size();i++){            
            std::cout << std::hex << (int)p_data[i] << " ";
        }
        std::cout << std::endl;
    }

    void MotorDriver::ALM_RST_Lite(bool is_echo = false, uint8_t id_ = 0x01){

        std::vector<uint8_t> p_data;
        uint8_t _num = 0x01;
        uint8_t _echo_bit = is_echo ? this->Echo_bit_Lite : this->No_Echo_bit_Lite;
        unionType _data_1, _data_2;
        _data_1._data = 0x00;
        _data_2._data = _echo_bit;

        p_data.clear();
        p_data.push_back(this->Broadcast);
        p_data.push_back(this->FC_MasterSendCMD_Lite);
        p_data.push_back(_num);
        p_data.push_back(id_);
        p_data.push_back(this->CMD_ALM_RST_Lite);
        p_data.push_back(_data_1._data_byte[1]);
        p_data.push_back(_data_1._data_byte[0]);
        p_data.push_back(_data_2._data_byte[1]);
        p_data.push_back(_data_2._data_byte[0]);
        uint16_t crc = this->calculate_CRC(p_data);
        p_data.push_back(crc);
        p_data.push_back(crc >> 8);
        std::vector<char> p_char(p_data.begin(), p_data.end());
        this->write(p_char);

        std::cout <<  "Send ALM_RST_Lite Command: ";
        for(auto i=0;i<p_char.size();i++){            
            std::cout << std::hex << (int)p_data[i] << " ";
        }
        std::cout << std::endl;
    }

    void MotorDriver::BRAKE_Lite(bool is_echo = false, uint8_t id_ = 0x01){

        std::vector<uint8_t> p_data;
        uint8_t _num = 0x01;
        uint8_t _echo_bit = is_echo ? this->Echo_bit_Lite : this->No_Echo_bit_Lite;
        unionType _data_1, _data_2;
        _data_1._data = 0x00;
        _data_2._data = _echo_bit;

        p_data.clear();
        p_data.push_back(this->Broadcast);
        p_data.push_back(this->FC_MasterSendCMD_Lite);
        p_data.push_back(_num);
        p_data.push_back(id_);
        p_data.push_back(this->CMD_BRAKE_Lite);
        p_data.push_back(_data_1._data_byte[1]);
        p_data.push_back(_data_1._data_byte[0]);
        p_data.push_back(_data_2._data_byte[1]);
        p_data.push_back(_data_2._data_byte[0]);
        uint16_t crc = this->calculate_CRC(p_data);
        p_data.push_back(crc);
        p_data.push_back(crc >> 8);
        std::vector<char> p_char(p_data.begin(), p_data.end());
        this->write(p_char);

        std::cout <<  "Send BRAKE_Lite Command: ";
        for(auto i=0;i<p_char.size();i++){            
            std::cout << std::hex << (int)p_data[i] << " ";
        }
        std::cout << std::endl;
    }

    void MotorDriver::NULL_Lite(bool is_echo = false, uint8_t id_ = 0x01){

        std::vector<uint8_t> p_data;
        uint8_t _num = 0x01;
        uint8_t _echo_bit = is_echo ? this->Echo_bit_Lite : this->No_Echo_bit_Lite;
        unionType _data_1, _data_2;
        _data_1._data = 0x00;
        _data_2._data = _echo_bit;

        p_data.clear();
        p_data.push_back(this->Broadcast);
        p_data.push_back(this->FC_MasterSendCMD_Lite);
        p_data.push_back(_num);
        p_data.push_back(id_);
        p_data.push_back(this->CMD_NULL_Lite);
        p_data.push_back(_data_1._data_byte[1]);
        p_data.push_back(_data_1._data_byte[0]);
        p_data.push_back(_data_2._data_byte[1]);
        p_data.push_back(_data_2._data_byte[0]);
        uint16_t crc = this->calculate_CRC(p_data);
        p_data.push_back(crc);
        p_data.push_back(crc >> 8);
        std::vector<char> p_char(p_data.begin(), p_data.end());
        this->write(p_char);

        std::cout <<  "Send NULL_Lite Command: ";
        for(auto i=0;i<p_char.size();i++){            
            std::cout << std::hex << (int)p_data[i] << " ";
        }
        std::cout << std::endl;
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
        // std:: cout << "=========\n";
        // std:: cout << std::dec << encoder_index_._data << " " << encoder_step_._data <<"\n";
        encoder_data_ = (encoder_index_._data + encoder_step_._data/10000.0)*2.0*M_PI;
        return encoder_data_;
    }

    double MotorDriver::get_Current(uint8_t id_){
        const int expected_bytes = 20;
        this->NULL_Lite(true, id_);
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

        double current_data_;

        union unionType current_bit_;
        current_bit_._data_byte[1]    = response.at(16);
        current_bit_._data_byte[0]    = response.at(17);
        // std:: cout << "=========\n";
        // std:: cout << std::dec << encoder_index_._data << " " << encoder_step_._data <<"\n";
        current_data_ = current_bit_._data*0.01;
        return current_data_;
    }

    double MotorDriver::get_Voltage(uint8_t id_){
        const int expected_bytes = 20;
        this->NULL_Lite(true, id_);
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

        double voltage_data_;

        union unionType voltage_bit_;
        voltage_bit_._data_byte[1]    = response.at(14);
        voltage_bit_._data_byte[0]    = response.at(15);
        // std:: cout << "=========\n";
        // std:: cout << std::dec << encoder_index_._data << " " << encoder_step_._data <<"\n";
        voltage_data_ = voltage_bit_._data*0.01;
        return voltage_data_;
    }


    // For driving Wheels.
    void MotorDriver::Multi_ISTOP_Lite(uint8_t Num_, std::vector<uint8_t> ID_, bool is_echo){

        std::vector<uint8_t> p_data;
        p_data.clear();
        p_data.push_back(this->Broadcast);
        p_data.push_back(this->FC_MasterSendCMD_Lite);
        p_data.push_back(Num_);


        for(auto i=0; i<ID_.size(); i++){
            unionType _data_i, _echo_i;
            _data_i._data = 0x00;
            _echo_i._data = is_echo ? this->Echo_bit_Lite : this->No_Echo_bit_Lite;
            p_data.push_back(ID_[i]);
            p_data.push_back(this->CMD_ISTOP_Lite);
            p_data.push_back(_data_i._data_byte[1]);
            p_data.push_back(_data_i._data_byte[0]);
            p_data.push_back(_echo_i._data_byte[1]);
            p_data.push_back(_echo_i._data_byte[0]);
        }

        uint16_t crc = this->calculate_CRC(p_data);
        p_data.push_back(crc);
        p_data.push_back(crc >> 8);
        std::vector<char> p_char(p_data.begin(), p_data.end());
        this->write(p_char);

        std::cout <<  "Send Multi_ISTOP_Lite Command: ";
        for(auto i=0;i<p_char.size();i++){            
            std::cout << std::hex << (int)p_data[i] << " ";
        }
        std::cout << std::endl;

    }

    void MotorDriver::Multi_JG_Lite(uint8_t Num_, std::vector<uint8_t> ID_, std::vector<int16_t> Data_,bool is_echo){

        std::vector<uint8_t> p_data;
        p_data.clear();
        p_data.push_back(this->Broadcast);
        p_data.push_back(this->FC_MasterSendCMD_Lite);
        p_data.push_back(Num_);

        for(auto i=0; i<ID_.size(); i++){
            unionType _data_i, _echo_i;
            _data_i._data = Data_[i];
            _echo_i._data = is_echo ? this->Echo_bit_Lite : this->No_Echo_bit_Lite;
            p_data.push_back(ID_[i]);
            p_data.push_back(this->CMD_JG_Lite);
            p_data.push_back(_data_i._data_byte[1]);
            p_data.push_back(_data_i._data_byte[0]);
            p_data.push_back(_echo_i._data_byte[1]);
            p_data.push_back(_echo_i._data_byte[0]);
        }

        uint16_t crc = this->calculate_CRC(p_data);
        p_data.push_back(crc);
        p_data.push_back(crc >> 8);
        std::vector<char> p_char(p_data.begin(), p_data.end());
        this->write(p_char);

        std::cout <<  "Send Multi_JG_Lite Command: ";
        for(auto i=0;i<p_char.size();i++){            
            std::cout << std::hex << (int)p_data[i] << " ";
        }
        std::cout << std::endl;
    }

    void MotorDriver::Multi_FREE_Lite(uint8_t Num_, std::vector<uint8_t> ID_, bool is_echo){

        std::vector<uint8_t> p_data;
        p_data.clear();
        p_data.push_back(this->Broadcast);
        p_data.push_back(this->FC_MasterSendCMD_Lite);
        p_data.push_back(Num_);

        for(auto i=0; i<ID_.size(); i++){
            unionType _data_i, _echo_i;
            _data_i._data = 0x00;
            _echo_i._data = is_echo ? this->Echo_bit_Lite : this->No_Echo_bit_Lite;
            p_data.push_back(ID_[i]);
            p_data.push_back(this->CMD_FREE_Lite);
            p_data.push_back(_data_i._data_byte[1]);
            p_data.push_back(_data_i._data_byte[0]);
            p_data.push_back(_echo_i._data_byte[1]);
            p_data.push_back(_echo_i._data_byte[0]);
        }

        uint16_t crc = this->calculate_CRC(p_data);
        p_data.push_back(crc);
        p_data.push_back(crc >> 8);
        std::vector<char> p_char(p_data.begin(), p_data.end());
        this->write(p_char);

        std::cout <<  "Send Multi_FREE_Lite Command: ";
        for(auto i=0;i<p_char.size();i++){            
            std::cout << std::hex << (int)p_data[i] << " ";
        }
        std::cout << std::endl;

    }

    void MotorDriver::Multi_SVON_Lite(uint8_t Num_, std::vector<uint8_t> ID_, bool is_echo){

        std::vector<uint8_t> p_data;
        p_data.clear();
        p_data.push_back(this->Broadcast);
        p_data.push_back(this->FC_MasterSendCMD_Lite);
        p_data.push_back(Num_);

        for(auto i=0; i<ID_.size(); i++){
            unionType _data_i, _echo_i;
            _data_i._data = 0x00;
            _echo_i._data = is_echo ? this->Echo_bit_Lite : this->No_Echo_bit_Lite;
            p_data.push_back(ID_[i]);
            p_data.push_back(this->CMD_SVON_Lite);
            p_data.push_back(_data_i._data_byte[1]);
            p_data.push_back(_data_i._data_byte[0]);
            p_data.push_back(_echo_i._data_byte[1]);
            p_data.push_back(_echo_i._data_byte[0]);
        }

        uint16_t crc = this->calculate_CRC(p_data);
        p_data.push_back(crc);
        p_data.push_back(crc >> 8);
        std::vector<char> p_char(p_data.begin(), p_data.end());
        this->write(p_char);

        std::cout <<  "Send Multi_SVON_Lite Command: ";
        for(auto i=0;i<p_char.size();i++){            
            std::cout << std::hex << (int)p_data[i] << " ";
        }
        std::cout << std::endl;

    }

    void MotorDriver::Multi_SVOFF_Lite(uint8_t Num_, std::vector<uint8_t> ID_, bool is_echo){

        std::vector<uint8_t> p_data;
        p_data.clear();
        p_data.push_back(this->Broadcast);
        p_data.push_back(this->FC_MasterSendCMD_Lite);
        p_data.push_back(Num_);

        for(auto i=0; i<ID_.size(); i++){
            unionType _data_i, _echo_i;
            _data_i._data = 0x00;
            _echo_i._data = is_echo ? this->Echo_bit_Lite : this->No_Echo_bit_Lite;
            p_data.push_back(ID_[i]);
            p_data.push_back(this->CMD_SVOFF_Lite);
            p_data.push_back(_data_i._data_byte[1]);
            p_data.push_back(_data_i._data_byte[0]);
            p_data.push_back(_echo_i._data_byte[1]);
            p_data.push_back(_echo_i._data_byte[0]);
        }

        uint16_t crc = this->calculate_CRC(p_data);
        p_data.push_back(crc);
        p_data.push_back(crc >> 8);
        std::vector<char> p_char(p_data.begin(), p_data.end());
        this->write(p_char);

        std::cout <<  "Send Multi_SVOFF_Lite Command: ";
        for(auto i=0;i<p_char.size();i++){            
            std::cout << std::hex << (int)p_data[i] << " ";
        }
        std::cout << std::endl;

    }

    void MotorDriver::Multi_ALM_RST_Lite(uint8_t Num_, std::vector<uint8_t> ID_, bool is_echo){

        std::vector<uint8_t> p_data;
        p_data.clear();
        p_data.push_back(this->Broadcast);
        p_data.push_back(this->FC_MasterSendCMD_Lite);
        p_data.push_back(Num_);

        for(auto i=0; i<ID_.size(); i++){
            unionType _data_i, _echo_i;
            _data_i._data = 0x00;
            _echo_i._data = is_echo ? this->Echo_bit_Lite : this->No_Echo_bit_Lite;
            p_data.push_back(ID_[i]);
            p_data.push_back(this->CMD_ALM_RST_Lite);
            p_data.push_back(_data_i._data_byte[1]);
            p_data.push_back(_data_i._data_byte[0]);
            p_data.push_back(_echo_i._data_byte[1]);
            p_data.push_back(_echo_i._data_byte[0]);
        }

        uint16_t crc = this->calculate_CRC(p_data);
        p_data.push_back(crc);
        p_data.push_back(crc >> 8);
        std::vector<char> p_char(p_data.begin(), p_data.end());
        this->write(p_char);

        std::cout <<  "Send Multi_ALM_RST_Lite Command: ";
        for(auto i=0;i<p_char.size();i++){            
            std::cout << std::hex << (int)p_data[i] << " ";
        }
        std::cout << std::endl;

    }

    void MotorDriver::Multi_BRAKE_Lite(uint8_t Num_, std::vector<uint8_t> ID_, bool is_echo){

        std::vector<uint8_t> p_data;
        p_data.clear();
        p_data.push_back(this->Broadcast);
        p_data.push_back(this->FC_MasterSendCMD_Lite);
        p_data.push_back(Num_);

        for(auto i=0; i<ID_.size(); i++){
            unionType _data_i, _echo_i;
            _data_i._data = 0x00;
            _echo_i._data = is_echo ? this->Echo_bit_Lite : this->No_Echo_bit_Lite;
            p_data.push_back(ID_[i]);
            p_data.push_back(this->CMD_BRAKE_Lite);
            p_data.push_back(_data_i._data_byte[1]);
            p_data.push_back(_data_i._data_byte[0]);
            p_data.push_back(_echo_i._data_byte[1]);
            p_data.push_back(_echo_i._data_byte[0]);
        }

        uint16_t crc = this->calculate_CRC(p_data);
        p_data.push_back(crc);
        p_data.push_back(crc >> 8);
        std::vector<char> p_char(p_data.begin(), p_data.end());
        this->write(p_char);

        std::cout <<  "Send Multi_BRAKE_Lite Command: ";
        for(auto i=0;i<p_char.size();i++){            
            std::cout << std::hex << (int)p_data[i] << " ";
        }
        std::cout << std::endl;

    }

    void MotorDriver::Multi_NULL_Lite(uint8_t Num_, std::vector<uint8_t> ID_, bool is_echo){

        std::vector<uint8_t> p_data;
        p_data.clear();
        p_data.push_back(this->Broadcast);
        p_data.push_back(this->FC_MasterSendCMD_Lite);
        p_data.push_back(Num_);

        for(auto i=0; i<ID_.size(); i++){
            unionType _data_i, _echo_i;
            _data_i._data = 0x00;
            _echo_i._data = is_echo ? this->Echo_bit_Lite : this->No_Echo_bit_Lite;
            p_data.push_back(ID_[i]);
            p_data.push_back(this->CMD_NULL_Lite);
            p_data.push_back(_data_i._data_byte[1]);
            p_data.push_back(_data_i._data_byte[0]);
            p_data.push_back(_echo_i._data_byte[1]);
            p_data.push_back(_echo_i._data_byte[0]);
        }

        uint16_t crc = this->calculate_CRC(p_data);
        p_data.push_back(crc);
        p_data.push_back(crc >> 8);
        std::vector<char> p_char(p_data.begin(), p_data.end());
        this->write(p_char);

        std::cout <<  "Send Multi_NULL_Lite Command: ";
        for(auto i=0;i<p_char.size();i++){            
            std::cout << std::hex << (int)p_data[i] << " ";
        }
        std::cout << std::endl;

    }


    // For steerng Wheels.
    void MotorDriver::Multi_ISTOP(uint8_t Num_, std::vector<uint8_t> ID_, bool is_echo){

        std::vector<uint8_t> p_data;
        p_data.clear();
        p_data.push_back(this->Broadcast);
        p_data.push_back(this->FC_MasterSendCMD);
        p_data.push_back(Num_);


        for(auto i=0; i<ID_.size(); i++){
            uint8_t _cmd = is_echo ? this->CMD_ISTOP_Echo : this->CMD_ISTOP_No_Echo;
            unionType _data_1, _data_2;
            _data_1._data = 0x00;
            _data_2._data = 0x00;
            p_data.push_back(ID_[i]);
            p_data.push_back(_cmd);
            p_data.push_back(_data_1._data_byte[1]);
            p_data.push_back(_data_1._data_byte[0]);
            p_data.push_back(_data_2._data_byte[1]);
            p_data.push_back(_data_2._data_byte[0]);
        }

        uint16_t crc = this->calculate_CRC(p_data);
        p_data.push_back(crc);
        p_data.push_back(crc >> 8);
        std::vector<char> p_char(p_data.begin(), p_data.end());
        this->write(p_char);

        std::cout <<  "Send Multi_ISTOP Command: ";
        for(auto i=0;i<p_char.size();i++){            
            std::cout << std::hex << (int)p_data[i] << " ";
        }
        std::cout << std::endl;

    }

    void MotorDriver::Multi_JG(uint8_t Num_, std::vector<uint8_t> ID_, std::vector<int16_t> Data_,bool is_echo){

        std::vector<uint8_t> p_data;
        p_data.clear();
        p_data.push_back(this->Broadcast);
        p_data.push_back(this->FC_MasterSendCMD);
        p_data.push_back(Num_);


        for(auto i=0; i<ID_.size(); i++){
            int8_t _cmd = is_echo ? this->CMD_JG_Echo : this->CMD_JG_No_Echo;
            unionType _data_1, _data_2;
            _data_1._data = 0x00;
            _data_2._data = Data_[i];
            p_data.push_back(ID_[i]);
            p_data.push_back(_cmd);
            p_data.push_back(_data_1._data_byte[1]);
            p_data.push_back(_data_1._data_byte[0]);
            p_data.push_back(_data_2._data_byte[1]);
            p_data.push_back(_data_2._data_byte[0]);
        }

        uint16_t crc = this->calculate_CRC(p_data);
        p_data.push_back(crc);
        p_data.push_back(crc >> 8);
        std::vector<char> p_char(p_data.begin(), p_data.end());
        this->write(p_char);

        std::cout <<  "Send Multi_JG Command: ";
        for(auto i=0;i<p_char.size();i++){            
            std::cout << std::hex << (int)p_data[i] << " ";
        }
        std::cout << std::endl;
    }

    void MotorDriver::Multi_FREE(uint8_t Num_, std::vector<uint8_t> ID_, bool is_echo){

        std::vector<uint8_t> p_data;
        p_data.clear();
        p_data.push_back(this->Broadcast);
        p_data.push_back(this->FC_MasterSendCMD);
        p_data.push_back(Num_);


        for(auto i=0; i<ID_.size(); i++){
            uint8_t _cmd = is_echo ? this->CMD_FREE_Echo : this->CMD_FREE_No_Echo;
            unionType _data_1, _data_2;
            _data_1._data = 0x00;
            _data_2._data = 0x00;
            p_data.push_back(ID_[i]);
            p_data.push_back(_cmd);
            p_data.push_back(_data_1._data_byte[1]);
            p_data.push_back(_data_1._data_byte[0]);
            p_data.push_back(_data_2._data_byte[1]);
            p_data.push_back(_data_2._data_byte[0]);
        }

        uint16_t crc = this->calculate_CRC(p_data);
        p_data.push_back(crc);
        p_data.push_back(crc >> 8);
        std::vector<char> p_char(p_data.begin(), p_data.end());
        this->write(p_char);

        std::cout <<  "Send Multi_FREE Command: ";
        for(auto i=0;i<p_char.size();i++){            
            std::cout << std::hex << (int)p_data[i] << " ";
        }
        std::cout << std::endl;

    }

    void MotorDriver::Multi_SVON(uint8_t Num_, std::vector<uint8_t> ID_, bool is_echo){

        std::vector<uint8_t> p_data;
        p_data.clear();
        p_data.push_back(this->Broadcast);
        p_data.push_back(this->FC_MasterSendCMD);
        p_data.push_back(Num_);


        for(auto i=0; i<ID_.size(); i++){
            uint8_t _cmd = is_echo ? this->CMD_SVON_Echo : this->CMD_SVON_No_Echo;
            unionType _data_1, _data_2;
            _data_1._data = 0x00;
            _data_2._data = 0x00;
            p_data.push_back(ID_[i]);
            p_data.push_back(_cmd);
            p_data.push_back(_data_1._data_byte[1]);
            p_data.push_back(_data_1._data_byte[0]);
            p_data.push_back(_data_2._data_byte[1]);
            p_data.push_back(_data_2._data_byte[0]);
        }

        uint16_t crc = this->calculate_CRC(p_data);
        p_data.push_back(crc);
        p_data.push_back(crc >> 8);
        std::vector<char> p_char(p_data.begin(), p_data.end());
        this->write(p_char);

        std::cout <<  "Send Multi_SVON Command: ";
        for(auto i=0;i<p_char.size();i++){            
            std::cout << std::hex << (int)p_data[i] << " ";
        }
        std::cout << std::endl;

    }

    void MotorDriver::Multi_SVOFF(uint8_t Num_, std::vector<uint8_t> ID_, bool is_echo){

        std::vector<uint8_t> p_data;
        p_data.clear();
        p_data.push_back(this->Broadcast);
        p_data.push_back(this->FC_MasterSendCMD);
        p_data.push_back(Num_);


        for(auto i=0; i<ID_.size(); i++){
            uint8_t _cmd = is_echo ? this->CMD_ISTOP_Echo : this->CMD_ISTOP_No_Echo;
            unionType _data_1, _data_2;
            _data_1._data = 0x00;
            _data_2._data = 0x00;
            p_data.push_back(ID_[i]);
            p_data.push_back(_cmd);
            p_data.push_back(_data_1._data_byte[1]);
            p_data.push_back(_data_1._data_byte[0]);
            p_data.push_back(_data_2._data_byte[1]);
            p_data.push_back(_data_2._data_byte[0]);
        }

        uint16_t crc = this->calculate_CRC(p_data);
        p_data.push_back(crc);
        p_data.push_back(crc >> 8);
        std::vector<char> p_char(p_data.begin(), p_data.end());
        this->write(p_char);

        std::cout <<  "Send Multi_ISTOP Command: ";
        for(auto i=0;i<p_char.size();i++){            
            std::cout << std::hex << (int)p_data[i] << " ";
        }
        std::cout << std::endl;

    }

    void MotorDriver::Multi_IMR(uint8_t Num_, std::vector<uint8_t> ID_, std::vector<int16_t> Index_, std::vector<uint16_t> Step_, bool is_echo){

        std::vector<uint8_t> p_data;
        p_data.clear();
        p_data.push_back(this->Broadcast);
        p_data.push_back(this->FC_MasterSendCMD);
        p_data.push_back(Num_);


        for(auto i=0; i<ID_.size(); i++){
            uint8_t _cmd = is_echo ? this->CMD_IMR_Echo : this->CMD_IMR_No_Echo;
            unionType _data_1, _data_2;
            _data_1._data = Index_[i];
            _data_2._data = Step_[i];
            p_data.push_back(ID_[i]);
            p_data.push_back(_cmd);
            p_data.push_back(_data_1._data_byte[1]);
            p_data.push_back(_data_1._data_byte[0]);
            p_data.push_back(_data_2._data_byte[1]);
            p_data.push_back(_data_2._data_byte[0]);
        }

        uint16_t crc = this->calculate_CRC(p_data);
        p_data.push_back(crc);
        p_data.push_back(crc >> 8);
        std::vector<char> p_char(p_data.begin(), p_data.end());
        this->write(p_char);

        std::cout <<  "Send Multi_IMR Command: ";
        for(auto i=0;i<p_char.size();i++){            
            std::cout << std::hex << (int)p_data[i] << " ";
        }
        std::cout << std::endl;

    }

    void MotorDriver::Multi_CS(uint8_t Num_, std::vector<uint8_t> ID_, std::vector<int16_t> Index_, std::vector<uint16_t> Step_, bool is_echo){

        std::vector<uint8_t> p_data;
        p_data.clear();
        p_data.push_back(this->Broadcast);
        p_data.push_back(this->FC_MasterSendCMD);
        p_data.push_back(Num_);


        for(auto i=0; i<ID_.size(); i++){
            uint8_t _cmd = is_echo ? this->CMD_CS_Echo : this->CMD_CS_No_Echo;
            unionType _data_1, _data_2;
            _data_1._data = Index_[i];
            _data_2._data = Step_[i];

            p_data.push_back(ID_[i]);
            p_data.push_back(_cmd);
            p_data.push_back(_data_1._data_byte[1]);
            p_data.push_back(_data_1._data_byte[0]);
            p_data.push_back(_data_2._data_byte[1]);
            p_data.push_back(_data_2._data_byte[0]);
        }

        uint16_t crc = this->calculate_CRC(p_data);
        p_data.push_back(crc);
        p_data.push_back(crc >> 8);
        std::vector<char> p_char(p_data.begin(), p_data.end());
        this->write(p_char);

        std::cout <<  "Send Multi_CS Command: ";
        for(auto i=0;i<p_char.size();i++){            
            std::cout << std::hex << (int)p_data[i] << " ";
        }
        std::cout << std::endl;

    }

    void MotorDriver::Multi_CMR(uint8_t Num_, std::vector<uint8_t> ID_, std::vector<int16_t> Index_, std::vector<uint16_t> Step_, bool is_echo){

        std::vector<uint8_t> p_data;
        p_data.clear();
        p_data.push_back(this->Broadcast);
        p_data.push_back(this->FC_MasterSendCMD);
        p_data.push_back(Num_);


        for(auto i=0; i<ID_.size(); i++){
            uint8_t _cmd = is_echo ? this->CMD_CMR_Echo : this->CMD_CMR_No_Echo;
            unionType _data_1, _data_2;
            _data_1._data = Index_[i];
            _data_2._data = Step_[i];
            p_data.push_back(ID_[i]);
            p_data.push_back(_cmd);
            p_data.push_back(_data_1._data_byte[1]);
            p_data.push_back(_data_1._data_byte[0]);
            p_data.push_back(_data_2._data_byte[1]);
            p_data.push_back(_data_2._data_byte[0]);
        }

        uint16_t crc = this->calculate_CRC(p_data);
        p_data.push_back(crc);
        p_data.push_back(crc >> 8);
        std::vector<char> p_char(p_data.begin(), p_data.end());
        this->write(p_char);

        std::cout <<  "Send Multi_CMR Command: ";
        for(auto i=0;i<p_char.size();i++){            
            std::cout << std::hex << (int)p_data[i] << " ";
        }
        std::cout << std::endl;

    }

    void MotorDriver::Multi_CMA(uint8_t Num_, std::vector<uint8_t> ID_, std::vector<int16_t> Index_, std::vector<uint16_t> Step_, bool is_echo){

        std::vector<uint8_t> p_data;
        p_data.clear();
        p_data.push_back(this->Broadcast);
        p_data.push_back(this->FC_MasterSendCMD);
        p_data.push_back(Num_);


        for(auto i=0; i<ID_.size(); i++){
            uint8_t _cmd = is_echo ? this->CMD_CMA_Echo : this->CMD_CMA_No_Echo;
            unionType _data_1, _data_2;
            _data_1._data = Index_[i];
            _data_2._data = Step_[i];
            p_data.push_back(ID_[i]);
            p_data.push_back(_cmd);
            p_data.push_back(_data_1._data_byte[1]);
            p_data.push_back(_data_1._data_byte[0]);
            p_data.push_back(_data_2._data_byte[1]);
            p_data.push_back(_data_2._data_byte[0]);
        }

        uint16_t crc = this->calculate_CRC(p_data);
        p_data.push_back(crc);
        p_data.push_back(crc >> 8);
        std::vector<char> p_char(p_data.begin(), p_data.end());
        this->write(p_char);

        std::cout <<  "Send Multi_CMA Command: ";
        for(auto i=0;i<p_char.size();i++){            
            std::cout << std::hex << (int)p_data[i] << " ";
        }
        std::cout << std::endl;

    }

    void MotorDriver::Multi_NULL(uint8_t Num_, std::vector<uint8_t> ID_, bool is_echo){

        std::vector<uint8_t> p_data;
        p_data.clear();
        p_data.push_back(this->Broadcast);
        p_data.push_back(this->FC_MasterSendCMD);
        p_data.push_back(Num_);


        for(auto i=0; i<ID_.size(); i++){
            uint8_t _cmd = is_echo ? this->CMD_NULL_Echo : this->CMD_NULL_No_Echo;
            unionType _data_1, _data_2;
            _data_1._data = 0x00;
            _data_2._data = 0x00;
            p_data.push_back(ID_[i]);
            p_data.push_back(_cmd);
            p_data.push_back(_data_1._data_byte[1]);
            p_data.push_back(_data_1._data_byte[0]);
            p_data.push_back(_data_2._data_byte[1]);
            p_data.push_back(_data_2._data_byte[0]);
        }

        uint16_t crc = this->calculate_CRC(p_data);
        p_data.push_back(crc);
        p_data.push_back(crc >> 8);
        std::vector<char> p_char(p_data.begin(), p_data.end());
        this->write(p_char);

        std::cout <<  "Send Multi_NULL Command: ";
        for(auto i=0;i<p_char.size();i++){            
            std::cout << std::hex << (int)p_data[i] << " ";
        }
        std::cout << std::endl;

    }

    bool MotorDriver::find_Steering_Home(uint8_t id_){
        /*
            尋 Home 應該要前、後輪分開做。
            * 1. 低速左轉：JG
            * 2. while 讀取IO-X1：NULL
            * 3. 低速右轉：JG
            * 4. while 讀取IO-X2：NULL
            * 5. 左轉固定角度：CMR
            * 6. 歸零：CS
        */

        const int expected_bytes = 20;
        int rpm = 300;
        this->JG(id_, rpm, false);
        usleep(10000);
        bool io_x1_state = false;
        while (!io_x1_state)
        {
            try
            {
                // get X1 IO & update to io_x1_state.
                
                this->NULL_Lite(true, id_);
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

                union unionType io_state;
                // io_state._data_byte[1] = response.at(12);
                io_state._data_byte[0] = response.at(13);
                io_x1_state = (io_state._data_byte[0]>>1)&1;
                usleep(10000);
            }
            catch(const std::exception& e)
            {
                std::cerr << e.what() << '\n';
            }
        }
        
        this->JG(id_, -rpm, false);
        usleep(10000);
        bool io_x2_state = false;
        while (!io_x2_state)
        {
            try
            {
                this->NULL_Lite(true, id_);
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

                union unionType io_state;
                // io_state._data_byte[1] = response.at(12);
                io_state._data_byte[0] = response.at(13);
                io_x2_state = (io_state._data_byte[0])&1;
                usleep(10000);
            }
            catch(const std::exception& e)
            {
                std::cerr << e.what() << '\n';
            }
        }
        this->JG(id_, 0, false);
        usleep(10000);
        this->CMR(id_, 20, 0, false);
        usleep(10000);
        this->CS(id_, 0, 0, false);
        usleep(10000);

        // 我覺得可以加入一個計時器，時間內沒有完成校正，就直接報錯。

        return true;
    }


}