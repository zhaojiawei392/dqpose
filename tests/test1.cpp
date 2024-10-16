
#include "../include/dqpose/quat.hpp"


int main() {
    using namespace dqpose;
    Quatf q0;
    Quatf q1{1,2,3,4};
    Quatf q2(q1);
    Quatf q3(Quatf(3));
    Quatf q4(4);


}