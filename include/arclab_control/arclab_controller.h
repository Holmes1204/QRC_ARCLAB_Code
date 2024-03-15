#include "basic_type.h"
class HWESTController;

class rl_controller
{
public:
    rl_controller();
    ~rl_controller();

    void getInput(proprioSense_t& feedback,gamepad_t& gamepad);
    void calculate();
    void getOutput(joint_cmd_t& cmd);
    void getDefault(joint_cmd_t& cmd);
private:
   HWESTController* controller;
    
};


