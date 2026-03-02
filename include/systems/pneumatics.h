#ifndef PNEUMATICS_H
#define PNEUMATICS_H

#include "main.h"
class Pneumatics{
    public:
    pros::adi::DigitalOut matchloader;
    pros::adi::DigitalOut hood;
    pros::adi::DigitalOut intake_piston;
    pros::adi::DigitalOut wing;
    pros::adi::DigitalOut intake_piston_bottom;
    Pneumatics(pros::adi::DigitalOut matchloader_port, pros::adi::DigitalOut intakepiston_port, pros::adi::DigitalOut wing_port,pros::adi::DigitalOut hood_port, pros::adi::DigitalOut intake_piston_bottom_port);

    void matchloader_initialize();
    void matchloader_control();
    void matchloader_v(int value);
    static int matchloader_task();
    
    void intakepiston_initialize();
    void intakepiston_control();
    void intakepiston_v(int value);
    static int intakepiston_task();

    void intake_piston_bottom_initialize();
    void intake_piston_bottom_control();
    void intake_piston_bottom_v(int value);
    static int intake_piston_bottom_task();


    void wing_initialize();
    void wing_control();
    void wing_v(int value);
    static int wing_task();

    void hood_initialize();
    void hood_control();
    void hood_v(int value);
    static int hood_task();
    
    static int awp_task();
    static int awp_task2();
    static int awp_task3();
    static int awp_task4();
    static int awp_task5();
    static int awp_task6();
    static int awp_task7();
    static int awp_task8();
    static int awp_task_new();
};

#endif // PNEUMATICS_H
