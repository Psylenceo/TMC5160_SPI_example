# TMC5160_SPI_example

Note: As of 7/11/2019, only use TMCStepper library Version 0.4.4. Lesser versions are missing register functions and cause the compiler to error out. Using version 0.4.5 causes the motor to oscilate badly. 

current version 0.1.2

Arduino example walking through all the settings and order of operation to set settings and registers of a TMC5160 using teemuatlut's TMCStepper library.

-TMC5160-BOB is the PDF datasheet for the breakout board manufactured by trinamics and that was used for creating and testing this code
-TMC5160_datasheet_rev_1.10 is the pdf datasheet for the TMC5160 itself.
-TMC5160_calculations.xlsx is an excel file from trinamics to assist with the calculations of driver parameters.
