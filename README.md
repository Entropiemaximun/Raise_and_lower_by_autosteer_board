# Raise_and_lower_by_autosteer_Board  USB and UDP   test with 5.7.2 Agopengps   


Lower and raise  with D2 D5 or  A1 A2 pin  with autosteer board  PCB V2 (  or similar )

 Example of solution  with D2 D5 in autosteer board  V2
 
 https://www.youtube.com/watch?v=T0ZdjZxdR2E


------------------------------------
UDP& USB: 

![change d2-5 a1-2](https://user-images.githubusercontent.com/88970536/226169202-a821cdea-a2df-4894-84c9-47cbe16aa7c6.png)

#define  Hyd_up A1  ///   define   the  pin  to up and down       can be   D2 D5 A1 A2 ...  like A1 in this line

#define  Hyd_down A2


  It an be switch    to 
  

#define  Hyd_up D2  ///   define   the  pin  to up and down       can be   D2 D5 A1 A2 ...  like A1 in this line

#define  Hyd_down D5

   This ino will replace the original autosteer Ino  on the Arduino nano.

----------------------------
 
Example of application 
 
 https://www.youtube.com/watch?v=u_cRjiCn22A
 
 
 
 --------------------------------------------------------
  The timing  of 255  delivers an infinite timing  to the next operation.
 
![screen](https://user-images.githubusercontent.com/88970536/226169083-d34481fc-06cf-4a41-8f5c-9b3e73b813ab.png)
in case of stop or release activ switch   it stop the action
