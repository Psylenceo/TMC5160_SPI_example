# TMC5160_SPI_example (Depreciated) (please see TMC5160_FULL_SPI_CMD_Example or individual function repos)

NOTE: As of 7/24/2019 I am going to start a new properly organized repo for this example as well as making branched repos for just certain features. The reason for this is that as of v0.1.6, this code only has an initial move, just starting to work out issues with the autotuning and the code is already 1000 lines long maybe 800 if you remove all the comments, folding brackets, and spaces between groupings of code. Which would make this code be insanely long and overwhelming for someone just getting into coding. Not to mention my commits, and github flow is not optimal. As I've just been commiting everything to the main branch and releasing it even though I'm still editing code. 

The plan is to make a new repo that is meant for the FULL SPI command structure, which will most likely evolve into a multi thousand line code, but there will be proper branching and merging along with commits and better tags. Also there will be secondary repos made that will essentially be the branches of the FULL SPI code, but will have the code required for that specific function.

current version 0.1.6

Arduino example walking through all the settings and order of operation to set settings and registers of a TMC5160 using teemuatlut's TMCStepper library.

-TMC5160-BOB is the PDF datasheet for the breakout board manufactured by trinamics and that was used for creating and testing this code

-TMC5160_datasheet_rev_1.10 is the pdf datasheet for the TMC5160 itself.

-TMC5160_calculations.xlsx is an excel file from trinamics to assist with the calculations of driver parameters.
