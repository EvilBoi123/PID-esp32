typedef struct {
    float kp;
    float ki;
    float kd;

    float setpoint;
    float integral;
    float prev_error;

    float output_min;
    float output_max;
} PIDController;

void pid_init(PIDController* pid, float kp, float ki, float kd, float min_out, float max_out);
float pid_compute(PIDController* pid, float input, float dt);
