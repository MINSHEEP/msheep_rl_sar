/* SPDX-License-Identifier: Apache-2.0 */
#include "custom_quad_imu.hpp"
#include <array>
#include <cmath>
#include <iostream>
#include <stdexcept>
void Require(bool ok,const char* text){if(!ok)throw std::runtime_error(text);}
int main()
{
    try
    {
        using namespace custom_quad::imu;
        // Fixed HI91 frame, generated with independent Python struct/binascii CRC implementation.
        const std::array<uint8_t,82> packet{90,165,76,0,244,230,145,0,0,0,0,0,0,0,210,4,0,0,0,0,0,0,0,0,0,0,0,0,128,63,0,0,180,66,0,0,0,0,0,0,52,195,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,128,63,0,0,0,0,0,0,0,0,0,0,0,0};
        Decoder decoder;const auto now=Clock::now();
        Require(!decoder.feed(packet.data(),11,now),"Partial frame published");
        Require(decoder.feed(packet.data()+11,packet.size()-11,now),"Valid HI91 frame rejected");
        auto sample=*decoder.latest();
        Require(sample.device_time==1234,"Wrong device timestamp");
        Require(std::abs(sample.angular_velocity_rad_s[0]-std::acos(-1.)/2)<1e-9,"Gyro degree/radian conversion");
        Require(std::abs(sample.angular_velocity_rad_s[2]+std::acos(-1.))<1e-9,"Gyro sign");
        Require(std::abs(sample.acceleration_m_s2[2]-9.8)<1e-9,"Acceleration conversion");
        const double half=std::sqrt(.5);
        const auto body=to_body(sample,{half,0,0,half});
        Require(std::abs(body.angular_velocity_rad_s[0])<1e-9 &&
                std::abs(body.angular_velocity_rad_s[1]-std::acos(-1.)/2)<1e-9,"IMU mounting rotation");
        Require(std::abs(body.projected_gravity[2]+1)<1e-9,"Projected gravity");
        auto corrupted=packet;corrupted[20]^=1;
        Require(!decoder.feed(corrupted.data(),corrupted.size(),Clock::now()),"CRC failure accepted");
        Require(decoder.latest()->received_at==now,"CRC failure refreshed timestamp");
        Require(decoder.feed(packet.data(),packet.size(),Clock::now()),"Resynchronization failed");
        std::cout<<"PASS: HI91 split frame, CRC rejection, resync, gyro units, mounting and gravity\n";
        return 0;
    }
    catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
