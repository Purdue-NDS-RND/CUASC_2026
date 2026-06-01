Hi Andrew,

These are a set of instructions to help you use our code. 

# Key Terminal Commands

cd - change directory
ls - list
mv - move a file/directory or rename
cat - outputs the contents of a file (concatenate)
rm - removes a file
rm -rf - removes a directory


# Key vim Commands

vim instructions.md - opens instructions.md in vim

Once you are inside a file, there are 2 modes you will mainly use: normal and insert. Normal allows you to navigate the file and enter vim commands to exit the file. Insert allows you to edit the file.

:q! - exits a file without saving
:x - exits a file with saving


# To run my code for the object localization mission

In Terminal 1:

cd ~/CUASC_2026/
./mavros_boot.sh device

In Terminal 2:

cd ~/CUASC_2026/
./set_stream_rate.sh
cd ~/src/vision_pipeline/vision_pipeline/
./vision_sh


The recorded data can be found in ~/CUASC_Mission_Data/
