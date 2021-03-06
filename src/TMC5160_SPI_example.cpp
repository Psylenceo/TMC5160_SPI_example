/******************************************************
    This example code shows all the options for a
    TMC5160 to be controlled from only SPI.
    The code will be broken up just a little bit
    to show the various registers that need to
    be set to get each function running.
    Currently this example works best on teemuatluts
    TMC stepper library version 4.4.
    If you try to use 4.5 the motor chatters at idle pretty badly
    as if the driver is pulsing the motor. In version 4.3 and older
    some of the functions do not work or values are sent to wrong registers.
    For version details see teemuatluts github page for info..
    https://github.com/teemuatlut/TMCStepper
    Code written by: Psylenceo
    Also Authour tends to use {} as code folding points.
    I have included the option to use an arduino uno or nano, but the code
    may become larger than the 328P on the nano or uno can handle.
    If that is the case use a more powerful processor, comment out some
    of the dialog options, or if I can figure it out I'll block off sections
    with a chip identifier if statement so that at compile time it will only
    compile what is needed and can fit on the 328P.
 *  *********************************************************************/
#include <Arduino.h>                          //Include the arduino header for use within platformIO
#include <TMCStepper.h>                       //header for tmcstepper library
#include <TMCStepper_UTILITY.h>               //header for thcstepper utility library

#define TMC5160_SPI_EXAMPLE_VERSION 0x000105  //v0.1.5

#define pi 3.1415926535                       //Pi is used in calculating the drivers initial pwm gradient for autotuning the motor

/****************************************************
   This code uses the arduino hardware SPI, but software SPI
   can be used, I just have not set it up.
 *****************************************************/
/* If using arduino mega*/
#define sck      52     //SPI clock
#define mosi     51     //master transmit out slave receive input
#define miso     50     //master receive input slave transmit out
#define ss       53     //chip select

/*if using arduino uno, or nano*/
/*#define sck      13     //SPI clock
  #define mosi     11     //master transmit out slave receive input
  #define miso     12     //master receive input slave transmit out
  #define ss       10     //chip select
*/

#define drv_en 7                //pin 7 of the arduino will be used for driver enable and disable can be a pin of your choice
#define sense_resistor .075     //Change this to the value of your sense resistor

#define supply_voltage 24       //change this to the voltage you are trying to run the driver chip and motor at

/********************************************************
   Example motor -> Kysan 1040118 17HD-B8X300-0.4A  -> http://kysanelectronics.com/Products/datasheet_display.php?recordID=8585
   Operating voltage-> 12 volts
   operating current -> .4A   (400mA)
   coil resistance ->  30 ohms
   coil inductance ->  37 mH
   Holding torque -> 26 Ncm (260 mNm)
   Rotor torque -> 35 gcm^2
   Degrees ->1.8
   All of that info was from the data sheet and some of it is shown on the product page
 **********************************************************/
#define motor_milliamps 400    //Milliamps specified on motor datasheet. Change to match your motor.
#define motor_voltage 12       //Motor operating voltage shown on datasheet. Change tp match your motor.
#define motor_resistance 30    //Motor coil resistance from datasheet. Change to match your motor.
#define motor_hold_torque 260  //Motor holding torque from datasheet. Change to match your motor. May need to calculate
#define motor_counts 200       //number of full steps per full rotation of motor. 360 / degrees = full step count

/****************************************************
   Now we need to define some of the base setting that
   we'll be using with the driver chip.
   This example will be using the drivers internal clock
   which runs at 12MHz.
   Also in the drivers data sheet, in the stealth chop section
   it talks about pwm freq, clock frequency, and good ranges of operation.
   to try and stay within 20kHz-50kHz range, with the the optimum pwm freq
   being in the green section for a given clock frequency and selected PWM frequency.
   Since this example is using the internall 12MHz clock and
   we want the widest adjustment range, we will be using
   pwm_freq = 2, for starting frequency of 35.1kHz.
 ***********************************************************/
#define drv_clock 12000000    //using internal clock, tie the clk pin low. If using a different freq, change this value to match
#define drv_chop_freq 35100   //drivers chop frequency set by pwm_freq and based on clk frequency. Change to match.
#define drv_decay_percent 70  //percentage of chopper standstill cycle time for lower power dissipation and upper frequency limit
#define drv_microstep_res 256 //number of micro steps per full step. 

/***************************************************************
   Motor count calculations, these are mostly if you are using this code to
   test on a stepper with a lead screw, attached to a belt, or other mechanical motion.
   These calculations will specifically be in the xtarget mm command or mm per second function
   since the program will need to know how counts on your motor actually move a reference point 1mm.
   Think of a stepper motor with a lead screw being used on the z axis of a 3d printer. You want to
   be able to tell the axis to move # mm's and it actually move that many mm's.
   To get these numbers is straight forward. Be in counts position command mode (i'll think of an easier name)
   and make sure you have a "zero" reference point. Then measure (preferably with calipers) the distance from
   your "zero" point to a know flat surface. Command the motor to move 1000 counts and hold position. Remeasure
   from your "zero" spot to the known flat surface. Then subract your original position from your new position
   and divide that value by 1000. This gets you how much your "zero" reference point will move per micro step count.
   Example, the Kysan motor I used as an example earlier is the original lead screw motor for makergear m2 printers.
   Using the procedure described above, each micro step of the motor will move a makergear m2 print bed spider arm
   .00015703125mm or .157 micrometers per micro step.
 *****************************************************************/
#define motor_mm_per_microstep .00015703125 //Change this value to match your motor!!!!

/************************************************************
   Now we need to calculate some important values for inital
   register settings in the driver. If you want to adjust any
   of hte register settings after initialization. You can change them
   below in the setup section.
   First we calculate the nominal amperage of the motor, this
   is important if we are using a voltage other than what the motor
   is rated for:
   For example 12 volt @ .4 amps = 4.8 watts if we use 24 volts to power the motor
   then 4.8 watts / 24 volts = .2 amps.
   This is then used to set the Irun and Ihold registers along with tuning of the chopper modes.
   Next we calculate the back emf constant of the motor which is used to calculate the
   initial gradient for stealth chop to ramp up at.
   Then we calculate the PWM gradient, which will give the autotune a starting point.
   We also need to calculate teh initial PWM offset.
   Finally we calculate the drivers PWM off time.
 ***********************************************************/

float  nominal_amps = ( ((motor_milliamps * motor_voltage) / supply_voltage) * 1.0 ),                           //calculate the voltage based curent
       cbemf = ( motor_hold_torque / (2 * nominal_amps) ),                                                      //calc the back emf constant of the motor
       really_small_number = ( (1 / drv_chop_freq) ),                                                           //testing to driv_toff to calc the number
       microsteps_per_rev = 51200/*( (motor_counts *  drv_microstep_res) )*/,                                   //testing to get the micro steps per rev calced
       drv_pwm_grad = ( (cbemf * 2 * pi * ( (drv_clock * 1.46) / (supply_voltage * microsteps_per_rev))) ),     //calculate the pwm gradient
       drv_pwm_ofs = ( (374 * motor_resistance * (nominal_amps / 1000)) / supply_voltage),                      //calculate teh pwm offest
       driv_toff = ( 3/*really_small_number * drv_clock / 10(((1 / drv_chop_freq) * (drv_decay_percent / 100) * .5) * drv_clock - 12) / 32*/ ); //calculatethe toff of the motor, currently does not calculate value

double sample,                              //value for when terminal samples values
       step_tune_match,                      //counting number of steps during stage 1 autotune
       GCONF_value,
       GCONF_temp_value,
       IOIN_value,
       IOIN_temp_value,
       OFFSET_READ_value,
       OFFSET_READ_temp_value,
       XACTUAL_value,
       XACTUAL_temp_value,
       XTARGET_value,
       XTARGET_temp_value;

float pwm_sum_base,                           //base pwm sum value to be tested in autotune to find optimal set points
      pwm_sum_tune;                           //pwm sum value while in the loop for set point optimization

uint32_t tstep_min = 1048575,                 //minimum tstep value to assist in velocity based mode change settings
         tstep_max = 0,                       //max value while motor is running
         SW_MODE_value,
         SW_MODE_temp_value,
         RAMP_STAT_value,
         RAMP_STAT_temp_value,
         XLATCH_value,
         XLATCH_temp_value,
         ENCMODE_value,
         ENCMODE_temp_value,
         TSTEP_value,
         TSTEP_temp_value,
         VACTUAL_value,
         VACTUAL_temp_value,
         XENC_value,
         XENC_temp_value,
         ENC_STATUS_value,
         ENC_STATUS_temp_value,
         ENC_LATCH_value,
         ENC_LATCH_temp_value,
         MSCNT_value,
         MSCNT_temp_value,
         MSCURACT_value,
         MSCURACT_temp_value,
         CHOPCONF_value,
         CHOPCONF_temp_value,
         DRV_STATUS_value,
         DRV_STATUS_temp_value,
         PWM_SCALE_value,
         PWM_SCALE_temp_value,
         PWM_AUTO_value,
         PWM_AUTO_temp_value,
         LOST_STEPS_value,
         LOST_STEPS_temp_value;

int autotune_optimized_up_cnt,                //variable to count how many times the up autotune has resulted in the optimal value of pwm sum
    autotune_optimized_dn_cnt,                //variable to count how many times the down autotune has resulted in optimal value of pwm sum
    autotune_average_optimize_cnt,            //variable for when we try to optimize the pwm sum for both up and down.
    short_stall = 4,                          //variable for tuning short circuit detection stall detect in stealth mode
    sgl = 0,                                  //variable for tuning stall guard
    cs,                                       //variable used during cs = irun stage of autotune
    run_current,                              //variable used during cs = irun stage of autotune
    GSTAT_value,
    GSTAT_temp_value,
    RAMP_MODE_value,
    RAMP_MODE_temp_value;

bool autotune_optimization_flag,              //this flag will go high once autotune optimization is complete or pwm_sum_tune is the lowest it can get for a given number auto tune loops
     stall_flag,                              //flag for when motor is stalled
     matched,                                 //"flag" indicating that CS and IRun are equal during autotune start up
     first_scan_flag;                         //flag set after 1st scan of the altered register read
      

/***********************************************************
   Using the example motor, this gives us results of:
   Nominal amps -> 200mA
   cbemf -> .65
   PWM gradient -> 58.2 (the register will round down to 58)
   PWM starting offset -> 93.5 (this one may round up or down)
   Toff -> 3.36 (so toff should be set to 3 or 4, may need some testing)

   Now that we have our initializers calculated we need to tell
   the arduino what driver we are using and and give the register points
   a prefix. This is needed, but is useful when multiple motors are
   used with a single CPU.
 **********************************************************/

TMC5160Stepper driver = TMC5160Stepper(ss, sense_resistor); //we are also telling the libray what pin is chip select and what the sense resistor value is.

/**********************************************************
   Prototypes of functions.
   This makes this area much smaller, because it lets us put
   the function and everything inside the function below the main loop
 ***********************************************************/

void base_calc_values(void);                     //prototype, but this will show what the calculations above result in as initial values
void read_registers(void);                       //this is just a prototype, but the function will call up all the readable registers from the driver
void read_GCONF_address(void);                   //prototype function to read only the specific register address from the TMC5160
void read_GSTAT_address(void);                   //prototype function to read only the specific register address from the TMC5160
void read_IOIN_address(void);                    //prototype function to read only the specific register address from the TMC5160
void read_OFFSET_READ_address(void);             //prototype function to read only the specific register address from the TMC5160
void read_TSTEP_address(void);                   //prototype function to read only the specific register address from the TMC5160
void read_RAMP_MODE_address(void);               //prototype function to read only the specific register address from the TMC5160
void read_XACTUAL_address(void);                 //prototype function to read only the specific register address from the TMC5160
void read_VACTUAL_address(void);                 //prototype function to read only the specific register address from the TMC5160
void read_XTARGET_address(void);                 //prototype function to read only the specific register address from the TMC5160
void read_SW_MODE_address(void);                 //prototype function to read only the specific register address from the TMC5160
void read_RAMP_STAT_address(void);               //prototype function to read only the specific register address from the TMC5160
void read_XLATCH_address(void);                  //prototype function to read only the specific register address from the TMC5160
void read_ENCMODE_address(void);                 //prototype function to read only the specific register address from the TMC5160
void read_X_ENC_address(void);                   //prototype function to read only the specific register address from the TMC5160
void read_ENC_STATUS_address(void);              //prototype function to read only the specific register address from the TMC5160
void read_ENC_LATCH_address(void);               //prototype function to read only the specific register address from the TMC5160
void read_MSCNT_address(void);                   //prototype function to read only the specific register address from the TMC5160
void read_MSCURACT_A_address(void);              //prototype function to read only the specific register address from the TMC5160
void read_MSCURACT_B_address(void);              //prototype function to read only the specific register address from the TMC5160
void read_CHOPCONF_address(void);                //prototype function to read only the specific register address from the TMC5160
void read_COOLCONF_address(void);                //prototype function to read only the specific register address from the TMC5160
void read_PWM_SCALE_address(void);               //prototype function to read only the specific register address from the TMC5160
void read_PWM_AUTO_address(void);                //prototype function to read only the specific register address from the TMC5160
void read_LOST_STEP_address(void);               //prototype function to read only the specific register address from the TMC5160
void read_DRV_STATUS_address(void);              //prototype function to read only the specific register address from the TMC5160
void read_motor_performance(void);               //prototype
void read_altered_registers(void);               //prototype to only display registers that have been altered or are reading different from scan to scan


void setup() {
  /* start up uart config as PC interface */{
    Serial.begin(115200);                   //serial com at 115200 baud
    while (!Serial);                        //wait for the arduino to detect an open com port
    Serial.println("Start...");             //com port is open, send 1st mesage
    Serial.println("");                     //add a new line to separate information
    base_calc_values();                     //readout the defined calculations
    delay(10000);                           //wait 20 seconds so user can read values
  }

  /* start up SPI, configure SPI, and axis IO interfacing*/{
    SPI.begin();                            //stat spi
    pinMode(sck, OUTPUT);                   //set sck pin as output for spi clock
    pinMode(ss, OUTPUT);                    //set ss pin as an out put for chip select
    pinMode(drv_en, OUTPUT);                //set drv enable pin as out put
  }

  digitalWrite(drv_en, LOW);                //enable the driver so that we can send the initial register values

  /*Initial settings for basic SPI command stepper drive no other functions enabled*/ {
    driver.begin();                         // start the tmc library

    /* Inital register read */ {
      Serial.println("");                     //add a new line to separate information
      Serial.println(F("Initial drive start up register reading."));
      read_registers();                       //Read all TMC5160 readable registers. Should read initial power presets or last configuration.
      delay(10000);                            //allow some reading time
    }

    /* Reseting drive faults and re-enabling drive */ {
      digitalWrite(drv_en, HIGH);             //disable drive to clear any start up faults
      delay(1000);                            //give the drive some time to clear faults
      digitalWrite(drv_en, LOW);              //re-enable drive, to start loading in parameters
      driver.toff(0);                         //attempt to clear ground faults on start-up
      driver.GSTAT(7);                        //clear gstat faults
      read_DRV_STATUS_address();
      read_GSTAT_address();
      delay(10000);
    }

    /* Set operation current limits */
    driver.rms_current(nominal_amps, 1);    //set Irun and Ihold for the drive

    /* short circuit monitoring */    {
      //driver.GLOBAL_SCALER(128);             //adjust this for the current scale you are using
      driver.diss2vs(0);                    //driver monitors for short to supply
      driver.s2vs_level(6);                 //lower values set drive to be very sensitive to low side voltage swings
      driver.diss2g(0);                     //driver to monitor for short to ground
      driver.s2g_level(6);                  //lower values set drive to be very sensitive to high side voltage swings

      read_CHOPCONF_address();
      delay(5000);
    }

    /* base GCONF settings for bare stepper driver operation*/    {
      driver.recalibrate(0);                //do not recalibrate the z axis
      driver.faststandstill(0);             //fast stand still at 65ms
      driver.en_pwm_mode(0);                //no silent step
      driver.multistep_filt(0);             //normal multistep filtering
      driver.shaft(0);                      //motor direction cw
      driver.small_hysteresis(0);           //step hysteresis set 1/16
      driver.stop_enable(0);                //no stop motion inputs
      driver.direct_mode(0);                //normal driver operation

      read_GCONF_address();
      delay(5000);
    }
  }

  /* minimum settings to to get a motor moving using SPI commands */{
    driver.tbl(2);                          //set blanking time to 24
    driver.toff(driv_toff);                 //pwm off time factor
    driver.pwm_freq(1);                     //pwm at 35.1kHz
    driver.pwm_autograd(0);
    driver.pwm_autoscale(0);
  }

  /************************************************************
    With just the minimum settings above, if you want to test motion
    using the TMC5160 in SPI mode, then all you have to do is set
    what ramp mode you want to test in
    what your start and stop velocities are going to be
    what your accel, deccel, and velocities ar going to be
    and what kind of units your serial commands are going to be
    RPM (rotations per minute) use rpm(####);
    RPS (rotations per second) use rps(####);
    mm/s (millimeters per second) use mm_per_sec(####);
    or just plain old counts where a command of 51200, will get you
    one full revolution on a 1.8 degree stepper motor.

    Note: using mm/s mode, you will first have to calculate you mm to counts
    scaling. Start with a reference point and a fixed surface, measure the distance
    then send a counts command of 1,000. Remeasure from you reference point to the
    fixed surface. Then subtract your starting distance from your distance at
    1,000 counts and divide this distance by 1,000. This will get you how many
    millimeters per microstep. This value will then need to be added in the define
    near the top of this code

    @ 12MHz clock, a 1.8 motor using 256 micro steps, driver output top speed is:
     7500 RPM / 125 RPS / 45,000 deg/sec / 1,277 mm/sec
     Which takes 1.5 second from 0 rpm to 7500 RPM at an AMAX setting of 65156.
      
  *************************************************************/

  /* Ramp mode (default)*/{
    driver.RAMPMODE(0);             //set ramp mode to positioning
    driver.VSTOP(10);              //set stop velocity to 10 steps/sec
    driver.VSTART(10);             //set start velocity to 10 steps/sec

    driver.V1(204800);               //midpoint velocity to  steps/sec ( steps/sec)
    driver.VMAX(204800);             //max velocity to  steps/sec ( steps/sec)

    driver.A1(51200);               //initial accel at  steps/sec2 ( steps/sec2)
    driver.AMAX(51200);             //max accel at  steps/sec2 ( steps/sec2)

    driver.DMAX(51200);             //max deccel  steps/sec2 ( steps/sec2)
    driver.D1(51200);               //mid deccel  steps/sec2 ( steps/sec2)
  }

  /*First motion*/{
    /*Now that all the minimum settings to get a TMC5160 are set, well read all the registers again, to check settings*/
    Serial.println("");             //add a new line to separate information
    Serial.println(F("First motion register reading to see if parameters were set."));
    read_registers();               //Read all TMC5160 readable registers. Should read initial power presets or last configuration.
    delay(10000);                   //wait 30 seconds to allow for reading settings

    /*Perform zero crossing calibration. No idea what it does, I just do it act as a starting point.*/
    digitalWrite(drv_en, HIGH);     //disable the driver to clear short circuit fault
    driver.recalibrate(1);          //Perform an initial zero crossing calibration
    delay(1000);                    //wait 1 second for calibration
    driver.recalibrate(0);          //finish initial zero crossing calibration
    digitalWrite(drv_en, LOW);      //enable the driver
    driver.GSTAT(7);            //clear gstat faults

    /*Read all registers again see that GSTAT has cleared, and to make sure some of the other faults are cleared as well.*/
    Serial.println("");             //add a new line to separate information
    Serial.println(F("First motion register reading to see if pre motion faults were cleared."));
    read_registers();               //Read all TMC5160 readable registers. Should read initial power presets or last configuration.
    delay(10000);                   //wait 30 seconds to allow for reading settings

    /*Now lets start the first actual move to see if everything worked, and to hear what the stepper sounds like.*/
    if (driver.position_reached() == 1) driver.XTARGET((100 / motor_mm_per_microstep));     //verify motor is at starting position, then move motor equivalent to 100mm
    while (driver.position_reached() == 0);                                                 //while in motion do nothing. This prevents the code from missing actions
    if (driver.position_reached() == 1) driver.XTARGET(0);                                  //verify motor is at position, then move motor back to starting position
    while (driver.position_reached() == 0);                                                 //while in motion do nothing. This prevents the code from missing actions
  }

  delay(5000);                     //a pause between operations, not needed but I wanted.

  /*Silent step and autotuning*/{
    /******************************************************
     * autotune procedure as per datasheet 
     * 
     * AT#1 motor needs to be in stand still and actual current scale (CS) = run current (Irun)
     *        if standstill reduction is enabled, perform an inital step to bring drive back to run current level,
     *          or set IHOLD = IRUN
     *        VS pin needs to be at supply voltage
     *    Recommended-> to use reduced standstill current IHOLD < IRUN to prevent low chopper frequencies for extended time while tuning
     *    Required duration ~130ms
     * 
     * AT#1 conditions will result in PWM_OFS_AUTO value
     * 
     * AT#2 move motor at a velocity the uses full run current and produces enough EMF that the drive can measure it. (Typically 60 -300 RPM) (typically 71583 - 357914 counts/sec)
     *      (1.5 * PWM_OFS_AUTO) < PWM_SCALE_SUM < (4 * PWM_OFS_AUTO)
     *      PWM_SCALE_SUM < 255
     *     Required -> a minimum of 8 FULL steps. For optimum PWM_GRAD_AUTO of < 50, upto 400 FULL steps may be needed.
     * 
     * AT#2 conditions will result in PWM_GRAD_AUTO value
     * 
     * 1) Set IHOLD = IRUN (this currently done at startup)
     * 2) adjust motor position to until CS = IRUN
     *    a) monitor CS to ensure it matches IRUN
     * 3) delay 250ms to give ample stage 1 time
     * 4) set velocity to 6 counts/s and perform a minimum of 10 full steps in 2 step increments with a stand still in between
     * 5) verify that PWM_SCALE_SUM is between 1.5 - 4 times what PWM_OFS_AUTO is and less than 255. Adjust until in range if possible.
     * 6) move motor between 8 and 400 full steps, or until PWM_GRAD_AUTO <= 50
     * 
     * If GLOBALSCALAR or VS is changed autotune will need to be redone.
     * 
     * Autotuning is current feedback (settings pwm_autoscale =1 and pwm_autograd =1)
     * Feed forward velocity controlledmode (pwm_autoscale =0)
     * 
     * Drive status Open load a and b during tuning  either indicate that current regulation did not reach max within the last few fullsteps (no motor or velocity > PWM limit)
     * 
     * PWM_SCALE_SUM can be used to check coil resistance.
     * 
     * While in motion, PWM_SCALE_SUM can be used as an indicator if full load motor current regulator can sustain the current or not
     * 
     * PWM_SCALE_AUTO will approach 0 during tuning.
    ******************************************************/
    
    /* stealth settings */ {
      driver.en_pwm_mode(1);                                                  // silent step enable
      driver.TPWMTHRS(75);                                                    //65 decent point to avoid skips and mechanical load noise near top when and while moving down
      //driver.pwm_reg(4);                                                      //responsiveness of stealth chop PWM amplitude adjustment
      //driver.pwm_lim(29);                                                     //sets the current jerk when switching between spread cycle and stealth chop
      driver.pwm_autoscale(0);                                                //automatic current scaling
      driver.pwm_autograd(0);                                                 //automatic tuning
      driver.pwm_ofs(drv_pwm_ofs);                                            //user calculated pwm amplitude offset
      driver.pwm_grad(drv_pwm_grad);                                          //velocity pwm gradual
      //driver.freewheel(0);                                                  //0 hold current action
    }
    
    /* Change short circuit detection level to super sensitive to act as stall detection during autotune and stealth chop */{
      driver.s2vs_level(short_stall);   //set the low side short circuit detection to be over sensitive to in an attempt act as a stallguard during stealth mode
    }

    
    driver.sgt(0);                          //stallguard sensitivity
    driver.sfilt(0);                        //stallguard filtering on
    driver.sg_stop(1);                      //stallguard event stop enabled

    read_registers();
    delay(10000);

    /* Initiate autotune */ {
      Serial.println(F("Starting stealth chop autotune"));
      driver.pwm_autograd(1);
      driver.pwm_autoscale(1);                                                //automatic current scaling
      delay(360);                                                             //delay for 2 full step cycles to allow settling
    }

    /* Autotune stage 1 (AT#1) */{
      cs = driver.cs_actual();                                                //store the value of cs
      run_current = driver.irun();                                            //store the value of irun
      driver.VMAX(6);                                                         //set drive to run motor at 6Hz to perform stage 1 autotune
      driver.AMAX(6);

      while( (cs != run_current) && matched != 1) {                           //adjust the motor until cs = irun
        driver.XTARGET(2);                                                    //move motor 1 micro step
        while(driver.position_reached() == 0);                                //wait until move complete
        delay(200);                                                           //wait for current settling
        cs = driver.cs_actual();                                              //store the value of cs
        run_current = driver.irun();                                          //store the value of irun
        if(cs == run_current && step_tune_match == 200) matched = 1;          //when cs = irun and there has been a series of step increases thenexit loop
        step_tune_match++;                                                    //increment until designated number of tunes have been completed
      }
    }

    Serial.println(F("Autotune stage 1 complete"));

    read_PWM_SCALE_address();
    read_PWM_AUTO_address();
    read_motor_performance();
    delay(10000);

    
    /* Autotune stage 2 (AT#2) */ {
      while (autotune_optimization_flag == 0) {
        driver.VMAX(6);                                                         //set drive to run motor at 6Hz to perform stage 1 autotune
        driver.AMAX(6);
        Serial.println(F("Starting motion forward"));
        delay(5000);
        if (driver.position_reached() == 1) driver.XTARGET((200 / motor_mm_per_microstep));   //if motor in position command a 200 mm move
        stall_flag = 0;
        sample = 0;                                                       //reset stall flag
        while (driver.position_reached() == 0) {
        /*****
         * softstop
         * sg_stop
         * low side overcurrent to detect stall s2vs_level
         * s2ga or s2gb and s2vsa or s2vsb become set, to clear disable and enable driver
         * 
         */
        
        /* Temporary if statement to test functionality until library is updated */
        if(sample == 100000){
          read_PWM_AUTO_address();
          read_motor_performance();
          sample = 0;
        }

        if( ( (driver.DRV_STATUS() & 0x00001000) == 1 || (driver.DRV_STATUS() & 0x00002000) == 1) && stall_flag == 0){   
          stall_flag =1;
          Serial.println(F("Motor short circuit stalled"));
          break;
        }

        if( (driver.status_sg() == 1 || driver.stallguard() == 1) ){
          Serial.println(F("Stall guard fault"));
          if(sgl == 63) {
            Serial.println(F("stall guard setting maxxed, halting operation"));
            while(1);
          }
          if(sgl > -64 && sgl < 64) sgl++;
          driver.sgt(sgl);
          read_RAMP_STAT_address();
        }
        
        /*if( (driver.s2vsa() == 1 || driver.s2vsb() == 1) && stall_flag == 0){   
          stall_flag =1;
          Serial.println(F("Motor short circuit stalled"));
          break;
        }*/
        if( driver.TSTEP() > tstep_max ) tstep_max = driver.TSTEP();        //while motor is moving, measure tstep max step time              
        if( driver.TSTEP() < tstep_min ) tstep_min = driver.TSTEP();        //while motor is moving, measure tstep min step time
        
        //use pwm sum to determine best speed for auto tuning, pwm sum needs to be as low as possible
        //add skip step detection as well. exit autotune on skipped step
        sample = sample + 1;
      }

      

      Serial.print(F("Max tstep time recorded -> "));
      Serial.println(tstep_max);                                            //display measured max step time  
      Serial.print(F("Min tstep time recorded -> "));
      Serial.println(tstep_min);
      Serial.println(F(""));                                            //display measured min step time

      /*if( stall_flag ==1 ){
        digitalWrite(drv_en, HIGH);                                         //disable in between tests to allow reseting of physical position if need be. And to allow changing of settings.
        if(short_stall >= 4 && short_stall <= 15) driver.s2vs_level(short_stall+1);       //increase low side short sensitivity setting, until only stalls trigger flag
        Serial.println(F("Low side short circuit sensitivity -> "));
        Serial.println(short_stall);
        digitalWrite(drv_en, LOW);                                          //disable in between tests to allow reseting of physical position if need be. And to allow changing of settings.
        stall_flag = 0;                                                     //reset stall flag
      }*/
      
      //add temp measuring during pause between up and down motions
      if (driver.position_reached() == 1) {
        delay(5000);                                                         //Hold at position for .5 seconds before going back to starting position
        Serial.println(F("Starting motion reverse"));
        driver.XTARGET(0);                                                  //move motor back to starting position
        stall_flag = 0;                                                     //ensure stall flag is reset
      while (driver.position_reached() == 0) {
        /* Temporary if statement to test functionality until library is updated */
          if( ( (driver.DRV_STATUS() & 0x00001000) == 1 || (driver.DRV_STATUS() & 0x00002000) == 1) && stall_flag == 0){   
          stall_flag =1;
          Serial.println(F("Motor short circuit stalled"));
          break;
        }
        
        if(driver.XACTUAL() == (sample / motor_mm_per_microstep)){
          Serial.print(F("Running RMS current ->"));
          Serial.println(driver.rms_current(), DEC);
          sample = (sample + 10);
          Serial.print(F("Lower value means high mechanical load ->"));
          Serial.println(driver.sg_result(), DEC);                                                            //display stallguard status bits, can be used to read motor temp or mechanical load
          Serial.println(F(""));
        }
        
        /*if( (driver.s2vsa() == 1 || driver.s2vsb() == 1) && stall_flag == 0){   
          stall_flag =1;
          Serial.println(F("Motor short circuit stalled"));
          break;
        }*/

          //include tstep reporting to tune for when best to implement stealth and spread cycle
          //use pwm sum to determine best speed for auto tuning, pwm sum needs to be as low as possible
          //add skip step detection as well. exit autotune on skipped step
        }
      }

      if(autotune_average_optimize_cnt == 5) autotune_optimization_flag = 1;      //after 5 loops of stable autotune, stall and skipped steps detection exit loop
    }
    }
    read_registers();                                                             //read all registers to see what changed and by how much
    Serial.println(F("autotune finished"));                                     

  }

  digitalWrite(drv_en, HIGH);  //disable in between tests to allow reseting of physical position if need be. And to allow changing of settings.

} // end of setup
//end of setup

void loop() {
  Serial.println("In loop");
  while (1);                    //debug message hold to know when program has exited setup routine.

} //end of loop
//end of loop

/************************************************************
   Actual base calc values function;
   This function sends the calculated initial values via uart
   to either the arduino monitor, a terminal program, or anything you want
   that uses uart.
   The use of the arduino F() function is critical! to memory usage, without using the F() function,
   all the serial data constant strings would eat up ALL of an arduino megas sram memory.
   With the F() function the over code uses less than 10% of the megas SRAM.
***************************************************************/
void base_calc_values(void) {
  Serial.print(F("Supply voltage set to -> "));
  Serial.println(supply_voltage);
  Serial.print(F("Motor rated milliamps -> "));
  Serial.println(motor_milliamps);
  Serial.print(F("Motor rated volts -> "));
  Serial.println(motor_voltage);
  Serial.print(F("Motor coil resistance -> "));
  Serial.println(motor_resistance);
  Serial.print(F("Motor rated holding torque in mNm -> "));
  Serial.println(motor_hold_torque);
  Serial.print(F("Driver clock frequency -> "));
  Serial.println(drv_clock);
  Serial.print(F("Calculated nominal amps based on motor wattage -> "));
  Serial.println(nominal_amps);
  Serial.print(F("Calculated back EMF constant -> "));
  Serial.println(cbemf);
  Serial.print(F("calculated velocity based PWM gradient -> "));
  Serial.println(drv_pwm_grad);
  Serial.print(F("Calculated initial PWM offset -> "));
  Serial.println(drv_pwm_ofs);
  Serial.print(F("Calculated PMW off time initializer -> "));
  Serial.println(driv_toff);
  Serial.print(F("Drive PWM_SCALE_SUM calculation -> "));
  Serial.println( (drv_pwm_ofs + drv_pwm_grad * .7488) );   //The .7488 = 256 * (step frequency / clock freq)
} //end of base calc
//end of base calc

/* Read registers is for accessing all registers on the drive and displaying their information */
void read_registers(void){
  /* General Configuration registers */{
    /* read off GCONF */
      read_GCONF_address();

    /* read of GSTAT */
      read_GSTAT_address();    

    /*read off IOIN input status */
      read_IOIN_address();

    /*read OFFSET_READ */
      read_OFFSET_READ_address();
  }

  /* Velocity dependant driver feature control registers */{
    /* read TSTEP. shows actual time time between each step*/
    read_TSTEP_address();
  }

  /* Ramp generator motion control registers */ {
    /* read RAMPMODE */
    read_RAMP_MODE_address();
    
    /* read XACTUAL */
    read_XACTUAL_address();

    /* read VACTUAL */
    read_VACTUAL_address();

    /* read XTARGET */
    read_XTARGET_address();
  }

  /* Ramp generator driver feature control registers */ {
    /* read SW_MODE */
     read_SW_MODE_address();

    /* read RAMPSTAT */
      read_RAMP_STAT_address();
    
    /* read XLATCH */
    read_XLATCH_address();
  }//display position x latch occurred

  /* Encoder registers */ {
    /*read ENCMODE */
    read_ENCMODE_address();

    /* read X_ENC */
    read_X_ENC_address();

    /* read ENC_STATUS */
    read_ENC_STATUS_address();

    /* read ENC_LATCH */
    read_ENC_LATCH_address();
  }

  /* Motor drive registers */ {
    /* read MSCNT */
    read_MSCNT_address();

    /*read MSCURACT phase A */
    read_MSCURACT_A_address();

    /* read MSCURACT phase B */
    read_MSCURACT_B_address();
  }

  /* Driver registers */ {
    /* read CHOPCONF */
    read_CHOPCONF_address();

    /* read COOLCONF */
      read_COOLCONF_address();

    /* read DRV_STATUS */ 
      read_DRV_STATUS_address();

    /* read PWM_SCALE */ 
      read_PWM_SCALE_address();

    /* read PWM_AUTO */ 
      read_PWM_AUTO_address();

    /* read LOST_STEPS */
    read_LOST_STEP_address();
  }

  /*display measured load values (current, motor temp, and mechanical load)*/
    read_motor_performance();
    
} // end of read register
//end of read register

void read_GCONF_address(void){
  /* read off GCONF */{
      Serial.println(F(""));
      Serial.print(F("GCONF -> "));
      Serial.println(driver.GCONF(), BIN);                                                        //display gconf registers as binary
      if (driver.recalibrate() == 0)Serial.println(F("--GCONF recalibrate inactive"));
      if (driver.recalibrate() == 1)Serial.println(F("--GCONF recalibrate active"));
      if (driver.faststandstill() == 0)Serial.println(F("--standstill timeout detection after 87ms"));
      if (driver.faststandstill() == 1)Serial.println(F("--standstill timeout detection after 21ms"));
      if (driver.en_pwm_mode() == 0)Serial.println(F("--stealthchop mode inactive"));
      if (driver.en_pwm_mode() == 1)Serial.println(F("--stealthchop mode active"));
      if (driver.multistep_filt() == 0)Serial.println(F("--external step input not filtered"));
      if (driver.multistep_filt() == 1)Serial.println(F("--external step input filtered"));
      if (driver.shaft() == 0)Serial.println(F("--motor cw"));
      if (driver.shaft() == 1)Serial.println(F("--motor ccw"));
      if (driver.sd_mode() == 1) {
        if (driver.diag0_error() == 0)Serial.println(F("--no errors"));
        if (driver.diag0_error() == 1)Serial.println(F("--error either Over temp, short to ground, or undervoltage"));
        if (driver.diag0_otpw() == 1)Serial.println(F("--Over temp pre-warning"));
        if (driver.diag0_stall() == 1)Serial.println(F("--motor stall detected"));
        if (driver.diag1_stall() == 1)Serial.println(F("--motor stall detected"));
        if (driver.diag1_index() == 1)Serial.println(F("--diag 1 used as encoder index output"));
        if (driver.diag1_onstate() == 1)Serial.println(F("--diag 1 active while chopper is active"));
        if (driver.diag1_steps_skipped() == 1)Serial.println(F("--diag 1 toggle with skipped steps detected in dcstep mode"));
      }
      if (driver.diag0_int_pushpull() == 0)Serial.println(F("--diag 0 is active low output"));
      if (driver.diag0_int_pushpull() == 1)Serial.println(F("--diag 0 is active high output"));
      if (driver.diag1_poscomp_pushpull() == 0)Serial.println(F("--diag 1 is active low output"));
      if (driver.diag1_poscomp_pushpull() == 1)Serial.println(F("--diag 1 is active high output"));
      if (driver.small_hysteresis() == 0)Serial.println(F("--step frequency hysteresis is 1/16"));
      if (driver.small_hysteresis() == 1)Serial.println(F("--step frequency hysteresis is 1/32"));
      if (driver.stop_enable() == 0)Serial.println(F("--Normal operation (no stop control inputs)"));
      if (driver.stop_enable() == 1)Serial.println(F("--ENCA input becomes a stop control input"));
      if (driver.direct_mode() == 0)Serial.println(F("--Normal operation"));
      if (driver.direct_mode() == 1)Serial.println(F("--weird motor current control"));
    }
} //end of read GCONF
//end of read GCONF

void read_GSTAT_address(void){
  /* read of GSTAT */{
      Serial.println(F(""));
      Serial.print(F("GSTAT -> "));
      Serial.println(driver.GSTAT(), BIN);                                                        //display gstat register as binary
      if (driver.reset() == 1)Serial.println(F("--driver has been reset. All values restored to factory defaults. Reload specific configuration settings"));
      if (driver.drv_err() == 1)Serial.println(F("--over temp or short circuit fault. read drv_status for details"));
      if (driver.uv_cp() == 1)Serial.println(F("--undervoltage on charge pump detected"));
    }
}//end of read GSTAT
//end of read GSTAT

void read_IOIN_address(void){
  /*read off IOIN input status */{
      Serial.println(F(""));
      Serial.print(F("IOIN -> "));
      Serial.println(driver.IOIN(), BIN);                                                         //display ioin register as binary
      if (driver.refl_step() == 1)Serial.println(F("--refL input active"));
      if (driver.refr_dir() == 1)Serial.println(F("--refR input active"));
      if (driver.encb_dcen_cfg4() == 1)Serial.println(F("--encoder b input active"));
      if (driver.enca_dcin_cfg5() == 1)Serial.println(F("--encoder a input active"));
      //if (driver.drv_enn_cfg6() == 1)Serial.println(F("--drive enable input is active"));
      //if (driver.enc_n_dco() == 1)Serial.println(F("--encoder index input active"));
      if (driver.sd_mode() == 1)Serial.println(F("--step / direction mode input active"));
      if (driver.swcomp_in() == 1)Serial.println(F("--sw comparator input active"));
      Serial.print(F("--version -> ")); Serial.println(driver.version());
    }
}//end of read IOIN
//end of read IOIN

void read_OFFSET_READ_address(void){
  /*read OFFSET_READ */{
      Serial.println(F(""));
      Serial.print(F("OFFSET_READ -> "));
      Serial.println(driver.OFFSET_READ(), HEX);
      Serial.print(F("--offset phase a -> "));
      Serial.println(((driver.OFFSET_READ() & 0xFF00) >> 8), DEC);
      Serial.print(F("--offset phase b -> "));
      Serial.println((driver.OFFSET_READ() & 0x00FF), DEC);
    }
} //end of read OFFSET_READ
//end of read OFFSET_READ

void read_TSTEP_address(void){
  /* read TSTEP. shows actual time time between each step*/
    Serial.println(F(""));
    Serial.print(F("TSTEP -> "));
    Serial.println(driver.TSTEP(), DEC);                                                        //time between steps
    //Serial.println(" ms");
} //end of read TSTEP
//end of read TSTEP

void read_RAMP_MODE_address(void){
  /* read RAMPMODE */
    Serial.println(F(""));
    switch (driver.RAMPMODE()) {                                                                //ready rampmode register and explain mode
      case (1): Serial.println(F("Velocity mode positive (VMAX and AMAX only)")); break;
      case (2): Serial.println(F("Velocity mode negative (VMAX and AMAX only)")); break;
      case (3): Serial.println(F("Hold mode (constant velocity)")); break;
      default: Serial.println(F("Positioning mode use all parameters")); break;
    }
} //end of read RAMPMODE
//end of read RAMPMODE

void read_XACTUAL_address(void){
  /* read XACTUAL */
  Serial.print(F("XACTUAL ->"));
  Serial.println(driver.XACTUAL(), DEC);                                                      //display actual count position

} //end of read XACTUAL
//end of read XACTUAL

void read_VACTUAL_address(void){
  /* read VACTUAL */
    Serial.print(F("VACTUAL ->"));
    Serial.println(driver.VACTUAL(), DEC);                                                      //actual velocity, most likely going to be 0 because this will be read with no motion in this register routine
    
} //end of read VACTUAL
//end of read VACTUAL

void read_XTARGET_address(void){
  /* read XTARGET */
    Serial.print(F("XTARGET ->"));
    Serial.println(driver.XTARGET(), DEC);                                                      //requested target position
} //end of read XTARGET
//end of read XTARGET

void read_SW_MODE_address(void){
  /* read SW_MODE */{
      Serial.println(F(""));
      Serial.print(F("SW_MODE ->"));
      Serial.println(driver.SW_MODE(), BIN);                                                      //read sw mode register, display as binary
      if (driver.en_softstop() == 1)Serial.println(F("--soft stop enabled. uses deccel ramp to stop and act as a limit switch"));
      if (driver.en_softstop() == 0)Serial.println(F("--soft stop disabled. uses physical limit switches"));
      if (driver.sg_stop() == 0)Serial.println(F("--stop by stallguard disabled"));
      if (driver.sg_stop() == 1)Serial.println(F("--stop by stallguard enabled"));
      if (driver.en_latch_encoder() == 1)Serial.println(F("--limit switch event stores encoder position into ENC_LATCH"));
      if (driver.latch_r_inactive() == 1)Serial.println(F("--right limit switch event latches on inactive signal (active / inactive signal set by pol_stop_r"));
      if (driver.latch_r_active() == 1)Serial.println(F("--right limit switch event latches on active signal (active / inactive signal set by pol_stop_r"));
      if (driver.latch_l_inactive() == 1)Serial.println(F("--left limit switch event latches on inactive signal (active / inactive signal set by pol_stop_l"));
      if (driver.latch_l_inactive() == 1)Serial.println(F("--left limit switch event latches on active signal (active / inactive signal set by pol_stop_l"));
      if (driver.swap_lr() == 0)Serial.println(F("--input on refL is left limit switch and refR is right limit switch"));
      if (driver.swap_lr() == 1)Serial.println(F("--input on refR is left limit switch and refL is right limit switch"));
      if (driver.pol_stop_r() == 0)Serial.println(F("--high signal on refR is considered active"));
      if (driver.pol_stop_r() == 1)Serial.println(F("--low signal on refR is considered active"));
      if (driver.pol_stop_l() == 0)Serial.println(F("--high signal on refL is considered active"));
      if (driver.pol_stop_l() == 1)Serial.println(F("--low signal on refL is considered active"));
      if (driver.stop_r_enable() == 0)Serial.println(F("--motor stops on refR active"));
      //if (driver.stop_l_enable() == 0)Serial.println(F("--motor stops on refL active"));
    }
} //end of read SW_MODE
//end of read SW_MODE

void read_RAMP_STAT_address(void){
  /* read RAMPSTAT */{
      Serial.println(F(""));
      Serial.print(F("RAMP_STAT ->"));
      Serial.println(driver.RAMP_STAT(), BIN);                                                    //display ramp status as binary
      if (driver.status_sg() == 1)Serial.println(F("--stall event detected"));
      if (driver.second_move() == 1)Serial.println(F("--ramp interrupted, reverse motion was required"));
      if (driver.t_zerowait_active() == 1)Serial.println(F("--standstill wait period active"));
      if (driver.vzero() == 1)Serial.println(F("--velocity is 0"));
      if (driver.position_reached() == 1)Serial.println(F("--position is reached"));
      if (driver.velocity_reached() == 1)Serial.println(F("--velocity reached"));
      if (driver.event_pos_reached() == 1)Serial.println(F("--target position reached"));
      if (driver.event_stop_sg() == 1)Serial.println(F("--stallguard event occurred"));
      if (driver.event_stop_r() == 1)Serial.println(F("--right limit switch event occurred"));
      if (driver.event_stop_l() == 1)Serial.println(F("--left limit switch event occurred"));
      if (driver.status_latch_r() == 0)Serial.println(F("--right latch not ready"));
      if (driver.status_latch_r() == 1)Serial.println(F("--right latch ready"));
      if (driver.status_latch_l() == 0)Serial.println(F("--left latch not ready"));
      if (driver.status_latch_l() == 1)Serial.println(F("--left latch ready"));
      if (driver.status_stop_r() == 0)Serial.println(F("--refR not active"));
      if (driver.status_stop_r() == 1)Serial.println(F("--refR active"));
      if (driver.status_stop_l() == 0)Serial.println(F("--refL not active"));
      if (driver.status_stop_l() == 1)Serial.println(F("--refL active"));
    }
} //end of read RAMP_STAT
//end of read RAMP_STAT

void read_XLATCH_address(void){
  /* read XLATCH */
    Serial.println(F(""));
    Serial.print(F("XLATCH ->"));
    Serial.println(driver.XLATCH(), DEC);
    //display position x latch occurred
} //end of read XLATCH
//end of read XLATCH

void read_ENCMODE_address(void){
  /*read ENCMODE */{
      Serial.println(F(""));
      Serial.print(F("ENCMODE ->"));
      Serial.println(driver.ENCMODE(), BIN);                                                      //display encmode as binary
      if (driver.enc_sel_decimal() == 0)Serial.println(F("--encoder constant binary divisor counts * value / 65536"));
      if (driver.enc_sel_decimal() == 1)Serial.println(F("--encoder constant decimal divisor counts * value / 10,000"));
      if (driver.latch_x_act() == 1)Serial.println(F("--latch XACTUAL with X_ENC"));
      if (driver.clr_enc_x() == 0)Serial.println(F("--index events latches X_ENC to ENC_LATCH"));
      if (driver.clr_enc_x() == 1)Serial.println(F("--index events latches X_ENC to ENC_LATCH and then clears X_ENC value"));
      if (driver.neg_edge() == 1)Serial.println(F("--N channel event triggers on inactive going N event"));
      if (driver.pos_edge() == 1)Serial.println(F("--N channel event triggers on active going N event"));
      if (driver.ignore_ab() == 0)Serial.println(F("--N event only occurs when pol N, pol a, pol b are correct polarity"));
      if (driver.ignore_ab() == 1)Serial.println(F("--disregard a pol and b pol pulses"));
      if (driver.pol_n() == 0)Serial.println(F("--N event on low pulse"));
      if (driver.pol_n() == 1)Serial.println(F("--N event on high pulse"));
      if (driver.pol_b() == 0)Serial.println(F("--b event on low pulse"));
      if (driver.pol_b() == 1)Serial.println(F("--b event on high pulse"));
      if (driver.pol_a() == 0)Serial.println(F("--a event on low pulse"));
      if (driver.pol_a() == 1)Serial.println(F("--a event on high pulse"));
    }
} //end of read ENCMODE
//end of read ENCMODE

void read_X_ENC_address(void){
  /* read X_ENC */
    Serial.println(F(""));
    Serial.print(F("Z_ENC ->"));
    Serial.println(driver.X_ENC(), DEC);
} //end of read X_ENC
//end of read X_ENC

void read_ENC_STATUS_address(void){
  /* read ENC_STATUS */
    Serial.print(F("ENC_STATUS ->"));
    Serial.println(driver.ENC_STATUS(), BIN);
} //end of ENC_STATUS
//end of read ENC_STATUS

void read_ENC_LATCH_address(void){
  /* read ENC_LATCH */
    Serial.print(F("ENC_LATCH ->"));
    Serial.println(driver.ENC_LATCH(), DEC);
} //end of ENC_LATCH
//end of ENC_LATCH

void read_MSCNT_address(void){
  /* read MSCNT */
    Serial.println(F(""));
    Serial.print(F("MSCNT ->"));
    Serial.println(driver.MSCNT(), DEC);
} //end of read MSCNT
//end of read MSCNT

void read_MSCURACT_A_address(void){
  /*read MSCURACT phase A */
    Serial.print(F("MSCURACT CUR_A ->"));
    Serial.println(driver.cur_a(), DEC);                                                        //current of phase a
} //end of read read_MSCURACT_A_address
//end of read MSCURACT_A

void read_MSCURACT_B_address(void){
  /* read MSCURACT phase B */
    Serial.print(F("MSCURACT CUR_B ->"));
    Serial.println(driver.cur_b(), DEC);                                                        //current of phase b
} //end of read read_MSCURACT_B
//end of read MSCURACT_B

void read_CHOPCONF_address(void){
  /* read CHOPCONF */{
      Serial.println(F(""));
      Serial.print(F("CHOPCONF ->"));
      Serial.println(driver.CHOPCONF(), BIN);                                                     //display chopconf as binary
      if (driver.diss2vs() == 0)Serial.println(F("--short to supply protection on"));
      if (driver.diss2vs() == 1)Serial.println(F("--short to supply protection off"));
      if (driver.diss2g() == 0)Serial.println(F("--short to ground protection on"));
      if (driver.diss2g() == 1)Serial.println(F("--short to ground protection off"));
      if (driver.dedge() == 0)Serial.println(F("--double edge step pulses disabled"));
      if (driver.dedge() == 1)Serial.println(F("--double edge step pulses enabled"));
      if (driver.intpol() == 0)Serial.println(F("--step pulse microstepping steps < 256"));
      if (driver.intpol() == 1)Serial.println(F("--step pulse microstepping steps at 256 per full step"));
      Serial.print(F("--multistep resolution ->"));
      Serial.println(driver.mres(), DEC);                                                         //display how many micro steps
      Serial.print(F("--Passive fast decay time ->"));   // compiler errors out
      Serial.println(driver.tpfd(), DEC);
      if (driver.vhighchm() == 0)Serial.println(F("--high velocity chopper disabled"));
      if (driver.vhighchm() == 1)Serial.println(F("--high velocity chopper enabled.  triggers when VHIGH is exceeded"));
      if (driver.vhighfs() == 0)Serial.println(F("--high velocity fullstep disabled"));
      if (driver.vhighfs() == 1)Serial.println(F("--high velocity fullstep enabled. triggers when VHIGH is exceeeded"));
      Serial.print(F("--PWM blanking time ->"));
      Serial.println(driver.tbl(), DEC);                                                          //display blanking time
      if (driver.chm() == 0)Serial.println(F("--Spread cycle active"));
      if (driver.chm() == 1)Serial.println(F("--constant fast decay mode"));
      if (driver.disfdcc() == 0)Serial.println(F("--current comparator termination enabled"));
      if (driver.disfdcc() == 1)Serial.println(F("--current comparator termination disabled"));
      if (driver.chm() == 0)Serial.print(F("--Hysteresis low value ->"));
      if (driver.chm() == 1)Serial.print(F("--Sine wave offset ->"));
      Serial.println(driver.hend(), DEC);                                                         //display hysteresis end value after selecting correct message based on chm
      if (driver.chm() == 0)Serial.print(F("--Hysteresis start value ->"));
      if (driver.chm() == 1)Serial.print(F("--Fst decay time ->"));
      Serial.println(driver.hstrt(), DEC);                                                        //display hysteresis start value after selecting correct message based on chm
      Serial.print(F("--Slow decay off time and driver enable ->"));
      Serial.println(driver.toff(), DEC);                                                         //display pwm off time setting
    }
} //end of read CHOPCONF
//end of read CHOPCONF

void read_COOLCONF_address(void){
  /* read COOLCONF */{
      Serial.println(F(""));
      Serial.print(F("COOLCONF ->"));
      Serial.println(driver.COOLCONF(), BIN);
      if (driver.sfilt() == 0)Serial.println(F("--standard stallguard filter timing"));
      if (driver.sfilt() == 1)Serial.println(F("--stallguard time filtered to measure every 4 full steps"));
      Serial.print(F("--Stallguard threshold ->"));
      Serial.println(driver.sgt(), DEC);                                                          //display stallguard threshold setting
      Serial.print(F("--current step down speed ->"));
      switch (driver.sedn()) {                                                                    //display current reduction rate
        case (1): Serial.println(F("8")); break;
        case (2): Serial.println(F("2")); break;
        case (3): Serial.println(F("1")); break;
        default: Serial.println(F("32")); break;
      }
      Serial.print(F("--smart current hysteresis ->"));
      Serial.println(driver.semax(), DEC);                                                         //display max setting for coolstep hysteresis
      Serial.print(F("--current step up speed ->"));
      switch (driver.seup()) {                                                                     //display current increase rate
        case (1): Serial.println(F("2")); break;
        case (2): Serial.println(F("4")); break;
        case (3): Serial.println(F("8")); break;
        default: Serial.println(F("1")); break;
      }
      Serial.print(F("--minimum smart current ->"));
      Serial.println(driver.semin(), DEC);
    }
} //end of read COOLCONF
//end of read COOLCONF

void read_DRV_STATUS_address(void){
  /* read DRV_STATUS */ {
      Serial.println(F(""));
      Serial.print(F("DRV_STATUS ->"));
      Serial.println(driver.DRV_STATUS(), BIN);                                                   //display driver status as binary
      if (driver.stst() == 1)Serial.println(F("--Standstill"));                                      // state what driver status bits are active
      if (driver.olb() == 1)Serial.println(F("--open load phase b"));
      if (driver.ola() == 1)Serial.println(F("--open load phase a"));
      if (driver.s2gb() == 1)Serial.println(F("--short to ground phase b"));
      if (driver.s2ga() == 1)Serial.println(F("--short to ground phase a"));
      if (driver.otpw() == 1)Serial.println(F("--over temp pre-warning"));
      if (driver.ot() == 1)Serial.println(F("--over temp"));
      if (driver.stallguard() == 1)Serial.println(F("--stall detected"));
      if (driver.fsactive() == 1)Serial.println(F("--full steps active"));
      //if(driver.stealth() == 1)Serial.println(F("--stealth chop is active"));             //library missing these 3 for tmc5160
      //if(driver.s2vsb() == 1)Serial.println(F("--short to vs phase b"));
      //if(driver.s2vsa() == 1)Serial.println(F("--short to vs phase a"));

      /* Temporary function read until library gets updated */
      if( (driver.DRV_STATUS() & 0x00004000) == 1)Serial.println(F("--stealth chop is active"));
      if( (driver.DRV_STATUS() & 0x00002000) == 1)Serial.println(F("--short to vs phase b"));
      if( (driver.DRV_STATUS() & 0x00001000) == 1)Serial.println(F("--short to vs phase a"));

    }
} //end of read DRV_STATUS
//end of read DRV_STATUS

void read_PWM_SCALE_address(void){
  /* read PWM_SCALE */ {
      Serial.println(F(""));
      Serial.print(F("PWM_SCALE_SUM ->"));
      Serial.println(driver.pwm_scale_sum(), DEC);                                                //display pwm scale sum
      Serial.print(F("PWM_SCALE_AUTO ->"));
      Serial.println(driver.pwm_scale_auto(), DEC);                                               //display pwm auto scale
    }
} //end of read PWM_SCALE
//end of read PWM_SCALE

void read_PWM_AUTO_address(void){
  /* read PWM_AUTO */ {
      Serial.print(F("PWM_OFS_AUTO ->"));
      Serial.println(driver.pwm_ofs_auto(), DEC);          //PWM_AUTO
      Serial.print(F("PWM_GRAD_AUTO ->"));
      Serial.println(driver.pwm_grad_auto(), DEC);
    }
} //end of read PWM_AUTO
//end of read PWM_AUTO

void read_LOST_STEP_address(void){
  /* read LOST_STEPS */{
    Serial.println(F(""));
    Serial.print(F("LOST_STEPS ->"));
    Serial.println(driver.LOST_STEPS(), DEC);                                                   //display # of lost steps, most likely 0 due to no motion
    }
}   //end of read LOST_STEP
//end of read LOST_STEP

void read_motor_performance(void){
  /*display measured load values (current, motor temp, and mechanical load)*/{
    Serial.print(F("Running RMS current ->"));
    Serial.println(driver.rms_current(), DEC);                                                    //display rms current
    if (driver.stst() == 0)Serial.print(F("Lower value means high mechanical load ->"));
    if (driver.stst() == 1)Serial.print(F("Motor temp at stand still ->"));
    Serial.println(driver.sg_result(), DEC);                                                            //display stallguard status bits, can be used to read motor temp or mechanical load
    Serial.println(F(""));
  }
} //end of read performance
//end of read performance

void read_altered_registers(void){
  GCONF_value = driver.GCONF();
  GSTAT_value = driver.GSTAT();
  IOIN_value = driver.IOIN();
  OFFSET_READ_value = driver.OFFSET_READ();
  TSTEP_value = driver.TSTEP();
  RAMP_MODE_value = driver.RAMPMODE();
  XACTUAL_value = driver.XACTUAL();
  VACTUAL_value = driver.VACTUAL();
  XTARGET_value = driver.XTARGET();
  SW_MODE_value = driver.SW_MODE();
  RAMP_STAT_value = driver.RAMP_STAT();
  XLATCH_value = driver.XLATCH();
  ENCMODE_value = driver.ENCMODE();
  XENC_value = driver.X_ENC();
  ENC_STATUS_value = driver.ENC_STATUS();
  ENC_LATCH_value = driver.ENC_LATCH();
  MSCNT_value = driver.MSCNT();
  MSCURACT_value = driver.MSCURACT();
  CHOPCONF_value = driver.CHOPCONF();
  DRV_STATUS_value = driver.DRV_STATUS();
  PWM_SCALE_value = driver.PWM_SCALE();
  PWM_AUTO_value = driver.PWM_AUTO();
  LOST_STEPS_value = driver.LOST_STEPS();

  if(GCONF_value != GCONF_temp_value) read_GCONF_address();
  if(GSTAT_value != GSTAT_temp_value) read_GSTAT_address();
  if(IOIN_value != IOIN_temp_value) read_IOIN_address();
  if(OFFSET_READ_value != OFFSET_READ_temp_value) read_OFFSET_READ_address();
  if(TSTEP_value != TSTEP_temp_value) read_TSTEP_address();
  if(RAMP_MODE_value != RAMP_MODE_temp_value) read_RAMP_MODE_address();
  if(XACTUAL_value != XACTUAL_temp_value) read_XACTUAL_address();
  if(VACTUAL_value != VACTUAL_temp_value) read_VACTUAL_address();
  if(XTARGET_value != XTARGET_temp_value) read_XTARGET_address();
  if(SW_MODE_value != SW_MODE_temp_value) read_SW_MODE_address();
  if(RAMP_STAT_value != RAMP_STAT_temp_value) read_RAMP_STAT_address();
  if(XLATCH_value != XLATCH_temp_value) read_XLATCH_address();
  if(ENCMODE_value != ENCMODE_temp_value) read_ENCMODE_address();
  if(XENC_value != XENC_temp_value) read_X_ENC_address();
  if(ENC_STATUS_value != ENC_STATUS_temp_value) read_ENC_STATUS_address();
  if(ENC_LATCH_value != ENC_LATCH_temp_value) read_ENC_LATCH_address();
  if(MSCNT_value != MSCNT_temp_value) read_MSCNT_address();
  if(MSCURACT_value != MSCURACT_temp_value) {
    read_MSCURACT_A_address();
    read_MSCURACT_B_address();
  }
  if(CHOPCONF_value != CHOPCONF_temp_value) read_CHOPCONF_address();
  if(DRV_STATUS_value != DRV_STATUS_temp_value) read_DRV_STATUS_address();
  if(PWM_SCALE_value != PWM_SCALE_temp_value) read_PWM_SCALE_address();
  if(PWM_AUTO_value != PWM_AUTO_temp_value) read_PWM_AUTO_address();
  if(LOST_STEPS_value != LOST_STEPS_temp_value) read_LOST_STEP_address();

  GCONF_temp_value = GCONF_value;
  GSTAT_temp_value = GSTAT_value;
  IOIN_temp_value = IOIN_value;
  OFFSET_READ_temp_value = OFFSET_READ_value;
  TSTEP_temp_value = TSTEP_value;
  RAMP_MODE_temp_value = RAMP_MODE_value;
  XACTUAL_temp_value = XACTUAL_value;
  VACTUAL_temp_value = VACTUAL_value;
  XTARGET_temp_value = XTARGET_value;
  SW_MODE_temp_value = SW_MODE_value;
  RAMP_STAT_temp_value = RAMP_STAT_value;
  XLATCH_temp_value = XLATCH_value;
  ENCMODE_temp_value = ENCMODE_value;
  XENC_temp_value = XENC_value;
  ENC_STATUS_temp_value = ENC_STATUS_value;
  ENC_LATCH_temp_value = ENC_LATCH_value;
  MSCNT_temp_value = MSCNT_value;
  MSCURACT_temp_value = MSCURACT_value;
  CHOPCONF_temp_value = CHOPCONF_value;
  DRV_STATUS_temp_value = DRV_STATUS_value;
  PWM_SCALE_temp_value = PWM_SCALE_value;
  PWM_AUTO_temp_value = PWM_AUTO_value;
  LOST_STEPS_temp_value = LOST_STEPS_value;
} //end altered registers
//end altered registers


//end of program