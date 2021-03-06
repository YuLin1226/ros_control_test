#include <motor_modbus/modbus.h>

#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H


namespace Motor
{
    class MotorDriver : public SerialModbus
    {
    private:
        std::shared_ptr<boost::mutex> p_func_mutex;

        const uint8_t Broadcast         = 0x00;
        const uint8_t MOTOR_ID          = 0x01; // this shouldn't be const, cuz it's different for different motors.
        const uint8_t FC_MasterSendCMD  = 0x65;

        const uint8_t Front_Drive_ID    = 0x01;
        const uint8_t Rear_Drive_ID     = 0x02;
        const uint8_t Front_Steer_ID    = 0x03;
        const uint8_t Rear_Steer_ID     = 0x04;

        const uint8_t CMD_ISTOP_Echo    = 0x00;
        const uint8_t CMD_ISTOP_No_Echo = 0x64;
        const uint8_t CMD_JG_Echo       = 0x0A;
        const uint8_t CMD_JG_No_Echo    = 0x6E;
        const uint8_t CMD_FREE_Echo     = 0x05;
        const uint8_t CMD_FREE_No_Echo  = 0x69;
        const uint8_t CMD_SVON_Echo     = 0x06;
        const uint8_t CMD_SVON_No_Echo  = 0x6A;
        const uint8_t CMD_SVOFF_Echo    = 0x07;
        const uint8_t CMD_SVOFF_No_Echo = 0x6B;
        const uint8_t CMD_IMR_Echo      = 0x0B;
        const uint8_t CMD_IMR_No_Echo   = 0x6F;
        const uint8_t CMD_CS_Echo       = 0x0E;
        const uint8_t CMD_CS_No_Echo    = 0x72;
        const uint8_t CMD_CMR_Echo      = 0x0F;
        const uint8_t CMD_CMR_No_Echo   = 0x73;
        const uint8_t CMD_CMA_Echo      = 0x10;
        const uint8_t CMD_CMA_No_Echo   = 0x74;
        const uint8_t CMD_NULL_Echo     = 0x63;
        const uint8_t CMD_NULL_No_Echo  = 0x77;
        
        

        const int rcv_size              =    8;


    public:
        MotorDriver(const std::string _serial_port, const int _baud_rate);
        virtual ~MotorDriver();

        void ISTOP(uint8_t id_, bool is_echo);
        void JG(uint8_t id_, int16_t _cmd_rpm, bool is_echo);
        void FREE(uint8_t id_, bool is_echo);
        void SVON(uint8_t id_, bool is_echo);
        void SVOFF(uint8_t id_, bool is_echo);
        void IMR(uint8_t id_, int16_t _index, uint16_t _step, bool is_echo);
        void CS(uint8_t id_, int16_t _index, uint16_t _step, bool is_echo);
        void CMR(uint8_t id_, int16_t _index, uint16_t _step, bool is_echo);
        void CMA(uint8_t id_, int16_t _index, uint16_t _step, bool is_echo);
        void NULL_TO_ECHO(uint8_t id_, bool is_echo);

        double get_Encoder();
        double get_Current(uint8_t id_);
        double get_Voltage(uint8_t id_);
        
    private:
        const uint8_t MOTOR_ID_Lite         = 0x01; // this shouldn't be const, cuz it's different for different motors.
        const uint8_t FC_MasterSendCMD_Lite = 0x41;
        const uint8_t CMD_ISTOP_Lite        = 0x00;
        const uint8_t CMD_JG_Lite           = 0x01;
        const uint8_t CMD_FREE_Lite         = 0x05;
        const uint8_t CMD_SVON_Lite         = 0x06;
        const uint8_t CMD_SVOFF_Lite        = 0x07;
        const uint8_t CMD_ALM_RST_Lite      = 0x08;
        const uint8_t CMD_BRAKE_Lite        = 0x09;
        const uint8_t CMD_NULL_Lite         = 0x63;
        const uint8_t Echo_bit_Lite         = 0x7F; 
        const uint8_t No_Echo_bit_Lite      = 0x00; 
        /*
        Echo data:
        1. Motor State
        2. Hall Encoder Count
        3. Motor Speed (rpm)
        4. Error Code
        5. I/O State
        6. Voltage (V)
        7. Current (A)
        */


    public:
        void ISTOP_Lite(bool is_echo, uint8_t id_);
        void JG_Lite(uint16_t _cmd_rpm, bool is_echo, uint8_t id_);
        void FREE_Lite(bool is_echo, uint8_t id_);
        void SVON_Lite(bool is_echo, uint8_t id_);
        void SVOFF_Lite(bool is_echo, uint8_t id_);
        void ALM_RST_Lite(bool is_echo, uint8_t id_);
        void BRAKE_Lite(bool is_echo, uint8_t id_);
        void NULL_Lite(bool is_echo, uint8_t id_);


    public:
        // For driving Wheels.
        void Multi_ISTOP_Lite(uint8_t Num_, std::vector<uint8_t> ID_, bool is_echo);

        void Multi_JG_Lite(uint8_t Num_, std::vector<uint8_t> ID_, std::vector<int16_t> Data_, bool is_echo);

        void Multi_FREE_Lite(uint8_t Num_, std::vector<uint8_t> ID_, bool is_echo);

        void Multi_SVON_Lite(uint8_t Num_, std::vector<uint8_t> ID_, bool is_echo);

        void Multi_SVOFF_Lite(uint8_t Num_, std::vector<uint8_t> ID_, bool is_echo);

        void Multi_ALM_RST_Lite(uint8_t Num_, std::vector<uint8_t> ID_, bool is_echo);

        void Multi_BRAKE_Lite(uint8_t Num_, std::vector<uint8_t> ID_, bool is_echo);

        void Multi_NULL_Lite(uint8_t Num_, std::vector<uint8_t> ID_, bool is_echo);


    public:
        // For steering Wheels.
        void Multi_ISTOP(uint8_t Num_, std::vector<uint8_t> ID_, bool is_echo);

        void Multi_JG(uint8_t Num_, std::vector<uint8_t> ID_, std::vector<int16_t> Data_, bool is_echo);

        void Multi_FREE(uint8_t Num_, std::vector<uint8_t> ID_, bool is_echo);

        void Multi_SVON(uint8_t Num_, std::vector<uint8_t> ID_, bool is_echo);

        void Multi_SVOFF(uint8_t Num_, std::vector<uint8_t> ID_, bool is_echo);

        void Multi_NULL(uint8_t Num_, std::vector<uint8_t> ID_, bool is_echo);

        void Multi_IMR(uint8_t Num_, std::vector<uint8_t> ID_, std::vector<int16_t> _Index, std::vector<uint16_t> _Step, bool is_echo);

        void Multi_CS(uint8_t Num_, std::vector<uint8_t> ID_, std::vector<int16_t> _Index, std::vector<uint16_t> _Step, bool is_echo);

        void Multi_CMR(uint8_t Num_, std::vector<uint8_t> ID_, std::vector<int16_t> _Index, std::vector<uint16_t> _Step, bool is_echo);

        void Multi_CMA(uint8_t Num_, std::vector<uint8_t> ID_, std::vector<int16_t> _Index, std::vector<uint16_t> _Step, bool is_echo);

        bool find_Steering_Home(uint8_t id_);
    };
} // namespace Motor

#endif