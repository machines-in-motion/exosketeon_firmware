// This class manages bno imu data recieved on the can bus
#include <FlexCAN_T4.h>
#include "params.h"
#include "sharedMemory.h"

class BnoManager{

    public:
        BnoManager(const uint32_t address) : imu_address_(address), quat_address_((address<<4)|CAN_QUAT_ADDRESS), status_address_((address<<4)|CAN_STATUS_ADDRESS) {};
        void process_response(const CAN_message_t &msg){
            if (msg.id == quat_address_){
                union{
                    struct{
                        int16_t q[4];
                    }data_;
                    uint8_t buffer[8];
                }parsed_message;

                for(unsigned i = 0; i < msg.len; i++){
                    parsed_message.buffer[i] = msg.buf[i]; 
                }
                quat[0] = float(parsed_message.data_.q[0])/uint_range;
                quat[1] = float(parsed_message.data_.q[1])/uint_range;
                quat[2] = float(parsed_message.data_.q[2])/uint_range;
                quat[3] = float(parsed_message.data_.q[3])/uint_range;

            }
            else if (msg.id == status_address_){
                union{
                    struct{
                        // float_t radError;
                        uint8_t quality;                    
                    }data_;
                    uint8_t buffer[1];
                }parsed_message;

                for(unsigned i = 0; i < 1; i++){
                    parsed_message.buffer[i] = msg.buf[i]; 
                }
                quality = parsed_message.data_.quality;
            };
        };
        
        void update_shm(float* shm_quat, float &shm_quality){
            for(unsigned i = 0; i < 4; i++){
                shm_quat[i] = quat[i];
            }
            shm_quality = (float)quality;
        }

        // Ideally these should be private
        float_t quat[4] = {0.,0.,0.,1.};
        float_t radError = 0.;
        uint8_t quality = 0; 

    private:
        const uint32_t imu_address_;
        const uint32_t quat_address_;
        const uint32_t status_address_;

};