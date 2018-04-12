#include "fanuc_control.h"



/*
==========================================================================
==========================================================================
ooooooooo.                       o8o      .    o8o                        
`888   `Y88.                     `"'    .o8    `"'                        
 888   .d88'  .ooooo.   .oooo.o oooo  .o888oo oooo   .ooooo.  ooo. .oo.   
 888ooo88P'  d88' `88b d88(  "8 `888    888   `888  d88' `88b `888P"Y88b  
 888         888   888 `"Y88b.   888    888    888  888   888  888   888  
 888         888   888 o.  )88b  888    888 .  888  888   888  888   888  
o888o        `Y8bod8P' 8""888P' o888o   "888" o888o `Y8bod8P' o888o o888o 
==========================================================================
==========================================================================
*/
Position::Position()
{
        x=0,     y=0,     z=0;
        w=0,     p=0,     r=0;
    conf1=0, conf2=0, conf3=0;
    turn1=0, turn2=0, turn3=0;
       motionType=0;
      motionSpeed=100;
    operationType=0;
    
    Validation();
    return;
}
Position::Position(double xtemp, double ytemp, double ztemp)
{
    x=xtemp, y=ytemp, z=ztemp;
    w=0,     p=0,     r=0;
    conf1=0, conf2=0, conf3=0;
    turn1=0, turn2=0, turn3=0;
       motionType=0;
      motionSpeed=100;
    operationType=0;

    Validation();
    return;
}
Position::Position(double xtemp, double ytemp, double ztemp,
                   double wtemp, double ptemp, double rtemp)
{
    x=xtemp, y=ytemp, z=ztemp;
    w=wtemp, p=ptemp, r=rtemp;
    conf1=0, conf2=0, conf3=0;
    turn1=0, turn2=0, turn3=0;
       motionType=0;
      motionSpeed=100;
    operationType=0;

    Validation();
    return;
}
Position::Position(double xtemp, double ytemp, double ztemp,
                   double wtemp, double ptemp, double rtemp,
                   int c1, int c2, int c3, int t1, int t2, int t3)
{
     x=xtemp,  y=ytemp,  z=ztemp;
     w=wtemp,  p=ptemp,  r=rtemp;
    conf1=c1, conf2=c2, conf3=c3;
    turn1=t1, turn2=t2, turn3=t3;
       motionType=0;
      motionSpeed=100;
    operationType=0;

    Validation();
    return;
}
Position::Position(double xtemp, double ytemp, double ztemp,
                   double wtemp, double ptemp, double rtemp,
                   int c1, int c2, int c3, int t1, int t2, int t3,
                   int mt, int ms, int ot)
{
     x=xtemp,  y=ytemp,  z=ztemp;
     w=wtemp,  p=ptemp,  r=rtemp;
    conf1=c1, conf2=c2, conf3=c3;
    turn1=t1, turn2=t2, turn3=t3;
       motionType=mt;
      motionSpeed=ms;
    operationType=ot;

    Validation();
    return;
}
/*
==============================================================================================
==============================================================================================
oooooo     oooo           oooo   o8o        .o8                .    o8o                        
 `888.     .8'            `888   `"'       "888              .o8    `"'                        
  `888.   .8'    .oooo.    888  oooo   .oooo888   .oooo.   .o888oo oooo   .ooooo.  ooo. .oo.   
   `888. .8'    `P  )88b   888  `888  d88' `888  `P  )88b    888   `888  d88' `88b `888P"Y88b  
    `888.8'      .oP"888   888   888  888   888   .oP"888    888    888  888   888  888   888  
     `888'      d8(  888   888   888  888   888  d8(  888    888 .  888  888   888  888   888  
      `8'       `Y888""8o o888o o888o `Y8bod88P" `Y888""8o   "888" o888o `Y8bod8P' o888o o888o
==============================================================================================
==============================================================================================
*/
void Position::Validation(void)
{
    // Correct motionType //
    if(this->motionType!=0 && this->motionType!=1)
    {
        yellow_txt("WARNING: "); white_txt("The value setted for motionType is not valid.\n");
        white_txt("The value must be ",1); yellow_txt("0 or 1!\n",1);
        white_txt("The value assumed will be 0 (Linear Motion)\n",1);
        white_txt("<Press Enter to Continue>",1);
        cin.ignore();
        this->motionType = 0;
    }
    // Correct operationType //
    if(this->operationType!=0 && this->operationType!=1)
    {
        yellow_txt("WARNING: "); white_txt("The value setted for operationType is not valid.\n");
        white_txt("The value must be ",1); yellow_txt("0 or 1!\n",1);
        white_txt("The value assumed will be 0 (synchronous)\n",1);
        white_txt("<Press Enter to Continue>",1);
        cin.ignore();
        this->operationType = 0;
    }
    // Correct motionSpeed //
    if((this->motionType==0 && this->motionSpeed < 1) ||
       (this->motionType==0 && this->motionSpeed > 4000))
    {
        yellow_txt("WARNING: "); white_txt("The value setted for motionSpeed is not valid.\n");
        white_txt("The value must be between ",1); yellow_txt("[1-4000]mm/s!\n",1);
        white_txt("The value assumed will be 100 (synchronous)\n,1");
        white_txt("<Press Enter to Continue>",1);
        cin.ignore();
        this->motionSpeed = 100;
    }else if((this->motionType==1 && this->motionSpeed < 1) ||
             (this->motionType==1 && this->motionSpeed > 100))
    {
        yellow_txt("WARNING: "); white_txt("The value setted for motionSpeed is not valid.\n");
        white_txt("The value must be between ",1); yellow_txt("[1-100]mm/s!\n",1);
        white_txt("The value assumed will be 10 (synchronous)\n",1);
        white_txt("<Press Enter to Continue>",1);
        cin.ignore();
        this->motionSpeed = 10;
    }
    // Means we are using joint type coordinates //
    if(this->motionType == 1)
    {
        this->conf2 = -60140;
        this->conf3 = -60140;
        this->turn1 = -60140;
        this->turn2 = -60140;
        this->turn3 = -60140;
    }

    return;
}
/*
==============================================================================================================
==============================================================================================================
ooooooooo.             o8o                  .   oooooo     oooo           oooo                                 
`888   `Y88.           `"'                .o8    `888.     .8'            `888                                 
 888   .d88' oooo d8b oooo  ooo. .oo.   .o888oo   `888.   .8'    .oooo.    888  oooo  oooo   .ooooo.   .oooo.o 
 888ooo88P'  `888""8P `888  `888P"Y88b    888      `888. .8'    `P  )88b   888  `888  `888  d88' `88b d88(  "8 
 888          888      888   888   888    888       `888.8'      .oP"888   888   888   888  888ooo888 `"Y88b.  
 888          888      888   888   888    888 .      `888'      d8(  888   888   888   888  888    .o o.  )88b 
o888o        d888b    o888o o888o o888o   "888"       `8'       `Y888""8o o888o  `V88V"V8P' `Y8bod8P' 8""888P' 
==============================================================================================================
==============================================================================================================
*/
void Position::PrintValues()
{
    if(this->motionType == 0)
    {
        green_txt("*********************************"); cout << endl;
        green_txt("*****    Variable Values    *****"); cout << endl;
        green_txt("*********************************"); cout << endl;
        green_txt("*** "); white_txt("x........................"); white_txt(Int2Str(this->x),1); cout << endl;
        green_txt("*** "); white_txt("y........................"); white_txt(Int2Str(this->y),1); cout << endl;
        green_txt("*** "); white_txt("z........................"); white_txt(Int2Str(this->z),1); cout << endl;
        green_txt("*** "); white_txt("w........................"); white_txt(Int2Str(this->w),1); cout << endl;
        green_txt("*** "); white_txt("p........................"); white_txt(Int2Str(this->p),1); cout << endl;
        green_txt("*** "); white_txt("r........................"); white_txt(Int2Str(this->r),1); cout << endl;
        green_txt("*** "); white_txt("conf1...................."); white_txt(Int2Str(this->conf1),1); cout << endl;
        green_txt("*** "); white_txt("conf2...................."); white_txt(Int2Str(this->conf2),1); cout << endl;
        green_txt("*** "); white_txt("conf3...................."); white_txt(Int2Str(this->conf3),1); cout << endl;
        green_txt("*** "); white_txt("turn1...................."); white_txt(Int2Str(this->turn1),1); cout << endl;
        green_txt("*** "); white_txt("turn2...................."); white_txt(Int2Str(this->turn2),1); cout << endl;
        green_txt("*** "); white_txt("turn3...................."); white_txt(Int2Str(this->turn3),1); cout << endl;
        green_txt("*** "); white_txt("motionType..............."); white_txt(Int2Str(this->motionType),1); cout << endl;
        green_txt("*** "); white_txt("motionSpeed.............."); white_txt(Int2Str(this->motionSpeed),1); cout << endl;
        green_txt("*** "); white_txt("operationType............"); white_txt(Int2Str(this->operationType),1); cout << endl;
        green_txt("*********************************"); cout << endl;
        green_txt("*********************************"); cout << endl;
        white_txt("<Press Enter to Continue>",1);
        cin.ignore();
    }else if(this->motionType == 1)
    {
        green_txt("*********************************"); cout << endl;
        green_txt("*****    Variable Values    *****"); cout << endl;
        green_txt("*********************************"); cout << endl;
        green_txt("*** "); white_txt("J1......................."); white_txt(Int2Str(this->x),1); cout << endl;
        green_txt("*** "); white_txt("J2......................."); white_txt(Int2Str(this->y),1); cout << endl;
        green_txt("*** "); white_txt("J3......................."); white_txt(Int2Str(this->z),1); cout << endl;
        green_txt("*** "); white_txt("J4......................."); white_txt(Int2Str(this->w),1); cout << endl;
        green_txt("*** "); white_txt("J5......................."); white_txt(Int2Str(this->p),1); cout << endl;
        green_txt("*** "); white_txt("J6......................."); white_txt(Int2Str(this->r),1); cout << endl;
        green_txt("*** "); white_txt("J7......................."); white_txt(Int2Str(this->conf1),1); cout << endl;
        green_txt("*** "); white_txt("motionType..............."); white_txt(Int2Str(this->motionType),1); cout << endl;
        green_txt("*** "); white_txt("motionSpeed.............."); white_txt(Int2Str(this->motionSpeed),1); cout << endl;
        green_txt("*** "); white_txt("operationType............"); white_txt(Int2Str(this->operationType),1); cout << endl;
        green_txt("*********************************"); cout << endl;
        green_txt("*********************************"); cout << endl;
        white_txt("<Press Enter to Continue>",1);
        cin.ignore();
    }
    return;
}
/*
========================================================================================================
========================================================================================================
  .oooooo.                                                  .     .oooo.    .oooooo..o     .            
 d8P'  `Y8b                                               .o8   .dP""Y88b  d8P'    `Y8   .o8            
888          ooo. .oo.   oooo    ooo  .ooooo.  oooo d8b .o888oo       ]8P' Y88bo.      .o888oo oooo d8b 
888          `888P"Y88b   `88.  .8'  d88' `88b `888""8P   888       .d8P'   `"Y8888o.    888   `888""8P 
888           888   888    `88..8'   888ooo888  888       888     .dP'          `"Y88b   888    888     
`88b    ooo   888   888     `888'    888    .o  888       888 . .oP     .o oo     .d8P   888 .  888     
 `Y8bood8P'  o888o o888o     `8'     `Y8bod8P' d888b      "888" 8888888888 8""88888P'    "888" d888b    
========================================================================================================
========================================================================================================
*/
string Position::Cnvert2Str(string type_of_conversion)
{
    /*********************************************************************************/
    /*********************************************************************************/
    /******        Type of Conversion        ||          Input Function          *****/
    /****** -------------------------------- || -------------------------------- *****/
    /****** [x y z w p r c1 c2 c3 t1 t2 t3   ||            "cart_all"            *****/
    /****** mt ms ot]                        ||                                  *****/
    /****** -------------------------------- || -------------------------------- *****/
    /******       [J1 J2 J3 J4 J5 J6]        ||           "joint_all"            *****/
    /****** -------------------------------- || -------------------------------- *****/
    /******          [x y z w p r]           ||           "check_cart"           *****/
    /****** -------------------------------- || -------------------------------- *****/
    /******      [J1 J2 J3 J4 J5 J6 J7]      ||           "check_joint"          *****/
    /******                                  ||                                  *****/
    /******                                  ||                                  *****/
    /******                                  ||                                  *****/
    /*********************************************************************************/
    /*********************************************************************************/

    string temp;
    if(type_of_conversion == "cart_all")
    {
        temp += Int2Str(this->x) + " " + Int2Str(this->y) + " " + Int2Str(this->z) + " ";
        temp += Int2Str(this->w) + " " + Int2Str(this->p) + " " + Int2Str(this->r) + " ";
        temp += Int2Str(this->conf1) + " " + Int2Str(this->conf2) + " " + Int2Str(this->conf3) + " ";
        temp += Int2Str(this->turn1) + " " + Int2Str(this->turn2) + " " + Int2Str(this->turn3) + " ";
        temp += Int2Str(this->motionType) + " " + Int2Str(this->motionSpeed) + " " + Int2Str(this->operationType);
    }
    if(type_of_conversion == "joint_all")
    {
        temp += Int2Str(this->x) + " " + Int2Str(this->y) + " " + Int2Str(this->z) + " ";
        temp += Int2Str(this->w) + " " + Int2Str(this->p) + " " + Int2Str(this->r) + " ";
        temp += Int2Str(this->motionType) + " " + Int2Str(this->motionSpeed) + " " + Int2Str(this->operationType);
    }
    if(type_of_conversion == "check_cart")
    {
        temp += Int2Str(this->x) + " " + Int2Str(this->y) + " " + Int2Str(this->z) + " ";
        temp += Int2Str(this->w) + " " + Int2Str(this->p) + " " + Int2Str(this->r);
    }
    if(type_of_conversion == "check_joint")
    {
        temp += Int2Str(this->x) + " " + Int2Str(this->y) + " " + Int2Str(this->z) + " ";
        temp += Int2Str(this->w) + " " + Int2Str(this->p) + " " + Int2Str(this->r);
    }

    //cout << temp;
    //cin.ignore();
    return temp;
}