 /***********
  * ***************************************************************************************
  * Software License Agreement (BSD License)
  *
  * Copyright (c) 2011-2014, LAR toolkit developers - University of Aveiro - http://lars.mec.ua.pt
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without modification, are permitted
  * provided that the following conditions are met:
  *
  *Redistributions of source code must retain the above copyright notice, this list of
  conditions and the following disclaimer.
  *Redistributions in binary form must reproduce the above copyright notice, this list of
  conditions and the following disclaimer in the documentation and/or other materials provided
  with the distribution.
  *Neither the name of the University of Aveiro nor the names of its contributors may be used to
  endorse or promote products derived from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
  IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
  FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
  IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
  OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  ***************************************************************************************************/


#include "../include/fanuc_control/fanuc_control.h"



/*
=============================
=============================
                         .o8  
                        "888  
oooo d8b  .ooooo.   .oooo888  
`888""8P d88' `88b d88' `888  
 888     888ooo888 888   888  
 888     888    .o 888   888  
d888b    `Y8bod8P' `Y8bod88P"
=============================
============================= 
*/
void red_txt(string msg)
{
    cout << "\033[1;31m" << msg << "\033[0m";
    return;
}
void red_txt(string msg, int dummy)
{
    cout << "\033[0;31m" << msg << "\033[0m";
    return;
}
/*
====================================================
====================================================
 .oooooooo oooo d8b  .ooooo.   .ooooo.  ooo. .oo.   
888' `88b  `888""8P d88' `88b d88' `88b `888P"Y88b  
888   888   888     888ooo888 888ooo888  888   888  
`88bod8P'   888     888    .o 888    .o  888   888  
`8oooooo.  d888b    `Y8bod8P' `Y8bod8P' o888o o888o 
d"     YD                                           
"Y88888P'                                           
====================================================
====================================================
*/
void green_txt(string msg)
{
    cout << "\033[1;32m" << msg << "\033[0m";
    return;
}
void green_txt(string msg, int dummy)
{
    cout << "\033[0;32m" << msg << "\033[0m";
    return;
}
/*
==============================================================
==============================================================
                      oooo  oooo                             
                      `888  `888                             
oooo    ooo  .ooooo.   888   888   .ooooo.  oooo oooo    ooo 
 `88.  .8'  d88' `88b  888   888  d88' `88b  `88. `88.  .8'  
  `88..8'   888ooo888  888   888  888   888   `88..]88..8'   
   `888'    888    .o  888   888  888   888    `888'`888'    
    .8'     `Y8bod8P' o888o o888o `Y8bod8P'     `8'  `8'     
.o..P'                                                       
`Y8P'                                                        
==============================================================
==============================================================
*/
void yellow_txt(string msg)
{
    cout << "\033[1;33m" << msg << "\033[0m";
    return;
}
void yellow_txt(string msg, int dummy)
{
    cout << "\033[0;33m" << msg << "\033[0m";
    return;
}
/*
=======================================
=======================================
 .o8       oooo                        
"888       `888                        
 888oooo.   888  oooo  oooo   .ooooo.  
 d88' `88b  888  `888  `888  d88' `88b 
 888   888  888   888   888  888ooo888 
 888   888  888   888   888  888    .o 
 `Y8bod8P' o888o  `V88V"V8P' `Y8bod8P' 
=======================================
=======================================
*/
void blue_txt(string msg)
{
    cout << "\033[1;36m" << msg << "\033[0m";
    return;
}
void blue_txt(string msg, int dummy)
{
    cout << "\033[0;36m" << msg << "\033[0m";
    return;
}
/*
=====================================================
=====================================================
                 oooo         o8o      .             
                 `888         `"'    .o8             
oooo oooo    ooo  888 .oo.   oooo  .o888oo  .ooooo.  
 `88. `88.  .8'   888P"Y88b  `888    888   d88' `88b 
  `88..]88..8'    888   888   888    888   888ooo888 
   `888'`888'     888   888   888    888 . 888    .o 
    `8'  `8'     o888o o888o o888o   "888" `Y8bod8P' 
=====================================================
=====================================================
*/
void white_txt(string msg)
{
    cout << "\033[1;37m" << msg << "\033[0m";
    return;
}
void white_txt(string msg, int dummy)
{
    cout << "\033[0;37m" << msg << "\033[0m";
    return;
}
/*
========================================================================================================================================================
========================================================================================================================================================
                            .o8                         .o88o.              .o88o.                                       .    o8o                        
                           "888                         888 `"              888 `"                                     .o8    `"'                        
 .ooooo.  ooo. .oo.    .oooo888               .ooooo.  o888oo              o888oo  oooo  oooo  ooo. .oo.    .ooooo.  .o888oo oooo   .ooooo.  ooo. .oo.   
d88' `88b `888P"Y88b  d88' `888              d88' `88b  888                 888    `888  `888  `888P"Y88b  d88' `"Y8   888   `888  d88' `88b `888P"Y88b  
888ooo888  888   888  888   888              888   888  888                 888     888   888   888   888  888         888    888  888   888  888   888  
888    .o  888   888  888   888              888   888  888                 888     888   888   888   888  888   .o8   888 .  888  888   888  888   888  
`Y8bod8P' o888o o888o `Y8bod88P" ooooooooooo `Y8bod8P' o888o   ooooooooooo o888o    `V88V"V8P' o888o o888o `Y8bod8P'   "888" o888o `Y8bod8P' o888o o888o
========================================================================================================================================================
======================================================================================================================================================== 
*/
void end_of_function(void)
{
    if(Global_debug_stop_functions)
    {
      cout << "End of Function" << endl;
      cout << "<Press Enter>";
      cin.ignore();
      cout << endl;
    }
    return;
}