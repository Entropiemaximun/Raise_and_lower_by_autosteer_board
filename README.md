# Raise_and_lower_by_autosteer_Board
Agopengps  V5.1.x or more   lower and raise  with D2 D5 or  A1 A2 pin  with autosteer board  PCB V2 (  or similar )

USB and UDP   test with 5.6.2

it  works only  if "Steer"  line is electrically activated on the pcb !

------------------------------------
UDP:  line 214

  #define  Hyd_up A1
  #define  Hyd_down A2
  can be switch 
  
  to 
  
  #define  Hyd_up 2
  #define  Hyd_down 5

------------------------------------
USP:  line 58

#define RAISE_PIN 2
#define LOWER_PIN 5

to

#define RAISE_PIN A1
#define LOWER_PIN A2
------------------------------------

download  the total folder  to  send  to the arduino nano
