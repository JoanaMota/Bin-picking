#include "../include/fanuc_control/fanuc_control.h"



/*
======================================================================
======================================================================
                    .o8         .oooooo.                               
                   "888        d8P'  `Y8b                              
oooo d8b  .ooooo.   888oooo.  888           .ooooo.  ooo. .oo.  .oo.   
`888""8P d88' `88b  d88' `88b 888          d88' `88b `888P"Y88bP"Y88b  
 888     888   888  888   888 888          888   888  888   888   888  
 888     888   888  888   888 `88b    ooo  888   888  888   888   888  
d888b    `Y8bod8P'  `Y8bod8P'  `Y8bood8P'  `Y8bod8P' o888o o888o o888o 
======================================================================
======================================================================
*/
robotCom::robotCom(boost::asio::io_service& io_service,
                   tcp::resolver::iterator endpoint_iterator)
:io_service_(io_service),
socket_(io_service)
{
    tcp::endpoint endpoint = *endpoint_iterator;
    socket_.async_connect(endpoint, boost::bind(&robotCom::handle_connect, this,boost::asio::placeholders::error, ++endpoint_iterator));
}
/*
==================================================================================================
==================================================================================================
                                            .                                                     
                                          .o8                                                     
ooo. .oo.  .oo.    .ooooo.  oooo    ooo .o888oo  .ooooo.   .ooooo.  oo.ooooo.   .ooooo.   .oooo.o 
`888P"Y88bP"Y88b  d88' `88b  `88.  .8'    888   d88' `88b d88' `"Y8  888' `88b d88' `88b d88(  "8 
 888   888   888  888   888   `88..8'     888   888   888 888        888   888 888   888 `"Y88b.  
 888   888   888  888   888    `888'      888 . 888   888 888   .o8  888   888 888   888 o.  )88b 
o888o o888o o888o `Y8bod8P'     `8'       "888" `Y8bod8P' `Y8bod8P'  888bod8P' `Y8bod8P' 8""888P' 
                                                                     888                          
                                                                    o888o                         
==================================================================================================
==================================================================================================
*/
void robotCom::movtocpos(string input)
{
    // Input must be like:
    //     MOVTOCPOS<CR>x y z w p r Cfg1 Cfg2 Cfg3 Turn1 Turn2 Turn3 motionType motionSpeed operationMode<CR>
            // x,y,z -- mm
            // w,p,r -- degrees
    //     EX: 100.00 200.00 300.00 10.00 20.00 30.00 0 1 1 0 0 0 1 100 0
    if(Global_debug_show_functions)
    {
        red_txt("*********************\n");
        red_txt("***   movtocpos   ***\n");
        red_txt("*********************\n");
        red_txt("Class:       ");red_txt("robotCom\n",1);
        white_txt("Function that will move the end effector to a given cartesian position.\n\n\n",1);
    }

    string data, temp1, temp2;
    // String Conversion to char* //
    char* temp = new char[input.size() + 1];
    copy(input.begin(), input.end(), temp);
    temp[input.size()] = '\0';

    // String concatunation //
    char msg[1024];
    strcpy(msg,"MOVTOCPOS\n");
    strcat(msg,temp);
    strcat(msg,"\n0\n");


    if(Global_debug_print_input_vars)
    {
        white_txt(msg);
        white_txt("<Press Enter>", 1);
        cin.ignore();
    }


    // Sends Order //
    write(msg,strlen(msg));


    boost::asio::streambuf response;
    istream response_stream(&response);

    boost::asio::read_until(socket_, response, "\n");
    getline(response_stream, data);
    boost::asio::read_until(socket_, response, "\n");
    getline(response_stream, temp1);


    if(Global_debug_show_results)
        cout << data << endl;


    end_of_function();
    return;
}
/*
===============================================================================================
===============================================================================================
                                            .                 o8o                               
                                          .o8                 `"'                               
ooo. .oo.  .oo.    .ooooo.  oooo    ooo .o888oo  .ooooo.     oooo oo.ooooo.   .ooooo.   .oooo.o 
`888P"Y88bP"Y88b  d88' `88b  `88.  .8'    888   d88' `88b    `888  888' `88b d88' `88b d88(  "8 
 888   888   888  888   888   `88..8'     888   888   888     888  888   888 888   888 `"Y88b.  
 888   888   888  888   888    `888'      888 . 888   888     888  888   888 888   888 o.  )88b 
o888o o888o o888o `Y8bod8P'     `8'       "888" `Y8bod8P'     888  888bod8P' `Y8bod8P' 8""888P' 
                                                              888  888                          
                                                          .o. 88P o888o                         
                                                          `Y888P                                
===============================================================================================
===============================================================================================
*/
void robotCom::movtojpos(string input)
{
    // Input must be like:
    //     J1 J2 J3 J4 J5 J6 Ty Sp Md
    //     EX:-1 -56 37 -1 -113 12 0 1000 1

    if(Global_debug_show_functions)
    {
        red_txt("*********************\n");
        red_txt("***   movtojpos   ***\n");
        red_txt("*********************\n");
        red_txt("Class:       ");red_txt("robotCom\n",1);
        white_txt("Function that will move the end effector to a given joint position.\n\n\n",1);
    }

    string data, temp1, temp2;
    // String Conversion to char* //
    char* temp = new char[input.size() + 1];
    copy(input.begin(), input.end(), temp);
    temp[input.size()] = '\0';

    // String concatunation //
    char msg[1024];
    strcpy(msg,"MOVTOJPOS\n");
    strcat(msg,temp);
    strcat(msg,"\n0\n");


    if(Global_debug_print_input_vars)
    {
        white_txt(msg);
        white_txt("<Press Enter>", 1);
        cin.ignore();
    }


    // Sends Order //
    write(msg,strlen(msg));


    boost::asio::streambuf response;
    istream response_stream(&response);

    boost::asio::read_until(socket_, response, "\n");
    getline(response_stream, data);
    boost::asio::read_until(socket_, response, "\n");
    getline(response_stream, temp1);
    boost::asio::read_until(socket_, response, "\n");
    getline(response_stream, temp2);

    // cout << msg << endl;

    if(Global_debug_show_results)
        cout << data << endl;


    end_of_function();
    return;
}
/*
=====================================================================================
=====================================================================================
                         .                          o8o                               
                       .o8                          `"'                               
 .oooooooo  .ooooo.  .o888oo  .ooooo.  oooo d8b    oooo oo.ooooo.   .ooooo.   .oooo.o 
888' `88b  d88' `88b   888   d88' `"Y8 `888""8P    `888  888' `88b d88' `88b d88(  "8 
888   888  888ooo888   888   888        888         888  888   888 888   888 `"Y88b.  
`88bod8P'  888    .o   888 . 888   .o8  888         888  888   888 888   888 o.  )88b 
`8oooooo.  `Y8bod8P'   "888" `Y8bod8P' d888b        888  888bod8P' `Y8bod8P' 8""888P' 
d"     YD                                           888  888                          
"Y88888P'                                       .o. 88P o888o                         
                                                `Y888P                               
=====================================================================================
===================================================================================== 
*/
string robotCom::getcrcpos(void)
{
    if(Global_debug_show_functions)
    {
        red_txt("*********************\n");
        red_txt("***   getcrcpos   ***\n");
        red_txt("*********************\n");
        red_txt("Class:       ");red_txt("robotCom\n",1);
        white_txt("Function that will return current cartesian position.\n\n\n",1);
    }



    string data, temp1,temp2, temp3, temp4, temp5;
    char msg[1000] = "GETCRCPOS\n";
    write(msg, strlen(msg));

    boost::asio::streambuf response;
    istream response_stream(&response);

    boost::asio::read_until(socket_, response, "\n");
    getline(response_stream, temp1);
    boost::asio::read_until(socket_, response, "\n");
    getline(response_stream, temp2);
    boost::asio::read_until(socket_, response, "\n");
    getline(response_stream, temp3);
    boost::asio::read_until(socket_, response, "\n");
    getline(response_stream, temp4);
    boost::asio::read_until(socket_, response, "\n");
    getline(response_stream, temp5);

    //TRANSFORMAR temp3 EM NUMEROS!
    

    m1 = temp2 + " " + temp3 + " " + temp4;

    if(Global_debug_show_functions)
    {
        white_txt(temp1 + "\n",1);
        white_txt(temp2 + "\n");
        white_txt(temp3 + "\n");
        white_txt(temp4 + "\n");
    }

    if(m1 == m2)
        Global_keep_going = false;
    else
        m2 = m1;

    end_of_function();
    return Cnvert2CSV(m1);
}
/*
=====================================================================================
=====================================================================================
                         .                          o8o                               
                       .o8                          `"'                               
 .oooooooo  .ooooo.  .o888oo  .ooooo.  oooo d8b    oooo oo.ooooo.   .ooooo.   .oooo.o 
888' `88b  d88' `88b   888   d88' `"Y8 `888""8P    `888  888' `88b d88' `88b d88(  "8 
888   888  888ooo888   888   888        888         888  888   888 888   888 `"Y88b.  
`88bod8P'  888    .o   888 . 888   .o8  888         888  888   888 888   888 o.  )88b 
`8oooooo.  `Y8bod8P'   "888" `Y8bod8P' d888b        888  888bod8P' `Y8bod8P' 8""888P' 
d"     YD                                           888  888                          
"Y88888P'                                       .o. 88P o888o                         
                                                `Y888P                               
=====================================================================================
===================================================================================== 
*/
string robotCom::getcrjpos(void)
{
    if(Global_debug_show_functions)
    {
        red_txt("*********************\n");
        red_txt("***   getcrjpos   ***\n");
        red_txt("*********************\n");
        red_txt("Class:       ");red_txt("robotCom\n",1);
        white_txt("Function that will current joint positions.\n\n\n",1);
    }

    string data, temp1,temp2, temp3;
    char msg[1000] = "GETCRJPOS\n";
    write(msg, strlen(msg));

    boost::asio::streambuf response;
    istream response_stream(&response);

    boost::asio::read_until(socket_, response, "\n");
    getline(response_stream, temp1);
    boost::asio::read_until(socket_, response, "\n");
    getline(response_stream, temp2);
    boost::asio::read_until(socket_, response, "\n");
    getline(response_stream, temp3);

    m3 = temp2;

    if(Global_debug_show_functions)
    {
        white_txt(temp1 + "\n",1);
        white_txt(temp2 + "\n");
    }

    if(m3 == m4)
        Global_keep_going = false;
    else
        m4 = m3;

    end_of_function();
    return Cnvert2CSV(m3);
}
/*
=============================================================================================
=============================================================================================
          oooo                            oooo                                                
          `888                            `888                                                
 .ooooo.   888 .oo.    .ooooo.   .ooooo.   888  oooo   .ooooo.  oo.ooooo.   .ooooo.   .oooo.o 
d88' `"Y8  888P"Y88b  d88' `88b d88' `"Y8  888 .8P'   d88' `"Y8  888' `88b d88' `88b d88(  "8 
888        888   888  888ooo888 888        888888.    888        888   888 888   888 `"Y88b.  
888   .o8  888   888  888    .o 888   .o8  888 `88b.  888   .o8  888   888 888   888 o.  )88b 
`Y8bod8P' o888o o888o `Y8bod8P' `Y8bod8P' o888o o888o `Y8bod8P'  888bod8P' `Y8bod8P' 8""888P' 
                                                                 888                          
                                                                o888o                        
=============================================================================================
=============================================================================================
*/
bool robotCom::checkcpos(string input)
{    
    if(Global_debug_show_functions)
    {
        red_txt("*********************\n");
        red_txt("***   checkcpos   ***\n");
        red_txt("*********************\n");
        red_txt("Class:       ");red_txt("robotCom\n",1);
        white_txt("Function that returns true if the given coordinates are in the workspace, and false otherwise.\n\n\n",1);
    }

    string data, temp1, temp2;
    // String Conversion to char* //
    char* temp = new char[input.size() + 1];
    copy(input.begin(), input.end(), temp);
    temp[input.size()] = '\0';

    // String concatunation //
    char msg[1024];
    strcpy(msg,"CHECKCPOS\n");
    strcat(msg,temp);
    strcat(msg,"\n");

    write(msg,strlen(msg));

    boost::asio::streambuf response;
    istream response_stream(&response);

    boost::asio::read_until(socket_, response, "\n");
    getline(response_stream, data);
    boost::asio::read_until(socket_, response, "\n");
    getline(response_stream, temp1);


    if(data == "PosOk")
    {
        white_txt("The cartesian coordinares ",1);
        white_txt(input);
        white_txt(" are in workspace.\n",1);
        if(temp1 == "1")
            white_txt("Operation was successful.\n");
        if(temp1 == "0")
            white_txt("Operation was not successful.\n");

        end_of_function();
        return true;
    }else if(data == "PosNok")
    {
        white_txt("The cartesian coordinares ",1);
        white_txt(input);
        white_txt(" are not in workspace.\n",1);
        if(temp1 == "1")
            white_txt("Operation was successful.\n");
        if(temp1 == "0")
            white_txt("Operation was not successful.\n");

        end_of_function();
        return false;
    }else
    {
        white_txt("Warning:");
        white_txt("message not recognized ---> ",1);
        white_txt(data + "\n");
        
        end_of_function();
        return false;
    }

}
/*
===========================================================================================
===========================================================================================

          oooo                            oooo            o8o                               
          `888                            `888            `"'                               
 .ooooo.   888 .oo.    .ooooo.   .ooooo.   888  oooo     oooo oo.ooooo.   .ooooo.   .oooo.o 
d88' `"Y8  888P"Y88b  d88' `88b d88' `"Y8  888 .8P'      `888  888' `88b d88' `88b d88(  "8 
888        888   888  888ooo888 888        888888.        888  888   888 888   888 `"Y88b.  
888   .o8  888   888  888    .o 888   .o8  888 `88b.      888  888   888 888   888 o.  )88b 
`Y8bod8P' o888o o888o `Y8bod8P' `Y8bod8P' o888o o888o     888  888bod8P' `Y8bod8P' 8""888P' 
                                                          888  888                          
                                                      .o. 88P o888o                         
                                                      `Y888P                                
===========================================================================================
===========================================================================================
*/
bool robotCom::checkjpos(string input)
{
    if(Global_debug_show_functions)
    {
        red_txt("*********************\n");
        red_txt("***   checkjpos   ***\n");
        red_txt("*********************\n");
        red_txt("Class:       ");red_txt("robotCom\n",1);
        white_txt("Function that returns true if the given coordinates are in the workspace, and false otherwise.\n\n\n",1);
    }

    string data, temp1, temp2;
    // String Conversion to char* //
    char* temp = new char[input.size() + 1];
    copy(input.begin(), input.end(), temp);
    temp[input.size()] = '\0';

    // String concatunation //
    char msg[1024];
    strcpy(msg,"CHECKJPOS\n");
    strcat(msg,temp);
    strcat(msg,"\n");

    write(msg,strlen(msg));

    boost::asio::streambuf response;
    istream response_stream(&response);

    boost::asio::read_until(socket_, response, "\n");
    getline(response_stream, data);
    boost::asio::read_until(socket_, response, "\n");
    getline(response_stream, temp1);

    cout << data << endl;
    cout << temp1 << endl;


    if(data == "Joint config is OK!")
    {
        white_txt("The joint coordinares ",1);
        white_txt(input);
        white_txt(" are in workspace.\n",1);
        if(temp1 == "1")
            white_txt("Operation was successful.\n");
        if(temp1 == "0")
            white_txt("Operation was not successful.\n");

        end_of_function();
        return  true;
    }else if(data == "Joint config out of reach")
    {
        white_txt("The joint coordinares ",1);
        white_txt(input);
        white_txt(" are not in workspace.\n",1);
        if(temp1 == "1")
            white_txt("Operation was successful.\n");
        if(temp1 == "0")
            white_txt("Operation was not successful.\n");

        end_of_function();
        return  false;
    }else
    {
        white_txt("Warning:");
        white_txt("message not recognized ---> ",1);
        white_txt(data);

        end_of_function();
        return false;
    }


}
/*
===========================================================
===========================================================
oooo   o8o               .       .              
`888   `"'             .o8     .o8              
 888  oooo   .oooo.o .o888oo .o888oo oo.ooooo.   oo.ooooo.  
 888  `888  d88(  "8   888     888    888' `88b  888' `88b 
 888   888  `"Y88b.    888     888    888   888  888   888 
 888   888  o.  )88b   888 .   888 .  888   888  888   888 
o888o o888o 8""888P'   "888"   "888"  888bod8P'  888bod8P' 
                                      888        888       
                                     o888o      o888o      
===========================================================
===========================================================
*/
void robotCom::listtpp(void)
{
    if(Global_debug_show_functions)
    {
        red_txt("*******************\n");
        red_txt("***   listtpp   ***\n");
        red_txt("*******************\n");
        red_txt("Class:     ");red_txt("robotCom\n",1);
        white_txt("Function that will return all console programms.\n\n\n",1);
    }



    cout << "***********************************************" << endl;
    cout << "***********************************************" << endl;
    cout << "***                                         ***" << endl;
    cout << "***   listpp is not working properly.       ***" << endl;
    cout << "***   the results cannot escape while cicle ***" << endl;
    cout << "***                                         ***" << endl;
    cout << "***********************************************" << endl;
    cout << "***********************************************" << endl;

/** ESTOU A FAZER SINCRONO E NAO DEVO 
AS LEITURAS VAO APARECER EM BACKGROUND E TENHO
QUE CRIAR UMA CALLBACK PARA LIDAR COM A LEITURA **/
/*
    string data;
    char msg[1000] = "LISTTPP\n";
    write(msg, strlen(msg));


    boost::asio::streambuf response;
    istream response_stream(&response);


    int bold = 0;
    for(int i=0; i<128; i++)
    {
        cout << "Begin \n";
        boost::asio::read_until(socket_, response, "\n");
        cout << "Now: \n";
        getline(response_stream, data);

        //if(data != "1")
        //{
            if(bold == 0) white_txt(data + " ");
            else white_txt(data +" ",1);
        //}

        cout << i << "                      THE END!\n";
        bold++;
    }


        //cout << "                   this is not msg !" << endl;

*/
    end_of_function();
    return;
}
/*
==============================================================
==============================================================
                                     .                         
                                   .o8                         
oooo d8b oooo  oooo  ooo. .oo.   .o888oo oo.ooooo.  oo.ooooo.  
`888""8P `888  `888  `888P"Y88b    888    888' `88b  888' `88b 
 888      888   888   888   888    888    888   888  888   888 
 888      888   888   888   888    888 .  888   888  888   888 
d888b     `V88V"V8P' o888o o888o   "888"  888bod8P'  888bod8P' 
                                          888        888       
                                         o888o      o888o     
==============================================================
============================================================== 
*/
void robotCom::runtpp(char *tppname)
{
    if(Global_debug_show_functions)
    {
        red_txt("******************\n");
        red_txt("***   runtpp   ***\n");
        red_txt("******************\n");
        red_txt("Class:    ");red_txt("robotCom\n",1);
        white_txt("Function that will run a console programm.\n\n\n",1);
    }

    string data;
    char msg[1000];

    strcpy(msg,"RUNTPP\n");
    strcat(msg,tppname);
    strcat(msg,"\n1\n");

    write(msg,strlen(msg));

    boost::asio::streambuf response;
    istream response_stream(&response);

    boost::asio::read_until(socket_, response, "\n");
    getline(response_stream, data);//On sucess it replies 1

    if(Global_debug_show_results)
    {
        if(data == "1")
        {
            blue_txt(tppname); blue_txt(" runned successfully!\n",1);
        }else
        {
            blue_txt(tppname); blue_txt(" did not run properly!\n",1);
        }
    }

    end_of_function();
    return;
}
/*
=====================================================================================================
=====================================================================================================
                                .    o8o                                     .                        
                              .o8    `"'                                   .o8                        
ooo. .oo.  .oo.    .ooooo.  .o888oo oooo   .ooooo.  ooo. .oo.    .oooo.o .o888oo  .ooooo.  oo.ooooo.  
`888P"Y88bP"Y88b  d88' `88b   888   `888  d88' `88b `888P"Y88b  d88(  "8   888   d88' `88b  888' `88b 
 888   888   888  888   888   888    888  888   888  888   888  `"Y88b.    888   888   888  888   888 
 888   888   888  888   888   888 .  888  888   888  888   888  o.  )88b   888 . 888   888  888   888 
o888o o888o o888o `Y8bod8P'   "888" o888o `Y8bod8P' o888o o888o 8""888P'   "888" `Y8bod8P'  888bod8P' 
                                                                                            888       
                                                                                           o888o     
=====================================================================================================
===================================================================================================== 
*/
void robotCom::motionstop(void)
{    
    if(Global_debug_show_functions)
    {
        red_txt("**********************\n");
        red_txt("***   motionstop   ***\n");
        red_txt("**********************\n");
        red_txt("Class:        ");red_txt("robotCom\n",1);
        white_txt("Function that stops the robot.\n\n\n",1);
    }

    string data, temp;
    char msg[1000];
    strcpy(msg,"MOTIONSTOP\n");
    write(msg,strlen(msg));

    boost::asio::streambuf response;
    istream response_stream(&response);

    boost::asio::read_until(socket_, response, "\n");
    getline(response_stream, data);//On sucess it replies 1
    if(Global_debug_show_results)
    {
        cout << data << endl;
    }

    end_of_function();
    return;
}
/*
=================================================
=================================================
                           o8o      .             
                           `"'    .o8             
oooo oooo    ooo oooo d8b oooo  .o888oo  .ooooo.  
 `88. `88.  .8'  `888""8P `888    888   d88' `88b 
  `88..]88..8'    888      888    888   888ooo888 
   `888'`888'     888      888    888 . 888    .o 
    `8'  `8'     d888b    o888o   "888" `Y8bod8P'
=================================================
================================================= 
*/
void robotCom::write(char* msg, int length)
{
    do_write(msg,length);
    return;
}
void robotCom::do_write(char*msg,int length)
{
    boost::asio::write(socket_,boost::asio::buffer(msg,length));
    return;
}
/*
=============================================
=============================================
          oooo                               
          `888                               
 .ooooo.   888   .ooooo.   .oooo.o  .ooooo.  
d88' `"Y8  888  d88' `88b d88(  "8 d88' `88b 
888        888  888   888 `"Y88b.  888ooo888 
888   .o8  888  888   888 o.  )88b 888    .o 
`Y8bod8P' o888o `Y8bod8P' 8""888P' `Y8bod8P' 
=============================================
=============================================
*/
void robotCom::close()
{
    do_close();
    return;
}
void robotCom::do_close()
{
    socket_.close();
    return;
}
/*
=================================================================================================================================================
=================================================================================================================================================
oooo                                    .o8  oooo                                                                                            .   
`888                                   "888  `888                                                                                          .o8   
 888 .oo.    .oooo.   ooo. .oo.    .oooo888   888   .ooooo.               .ooooo.   .ooooo.  ooo. .oo.   ooo. .oo.    .ooooo.   .ooooo.  .o888oo 
 888P"Y88b  `P  )88b  `888P"Y88b  d88' `888   888  d88' `88b             d88' `"Y8 d88' `88b `888P"Y88b  `888P"Y88b  d88' `88b d88' `"Y8   888   
 888   888   .oP"888   888   888  888   888   888  888ooo888             888       888   888  888   888   888   888  888ooo888 888         888   
 888   888  d8(  888   888   888  888   888   888  888    .o             888   .o8 888   888  888   888   888   888  888    .o 888   .o8   888 . 
o888o o888o `Y888""8o o888o o888o `Y8bod88P" o888o `Y8bod8P' ooooooooooo `Y8bod8P' `Y8bod8P' o888o o888o o888o o888o `Y8bod8P' `Y8bod8P'   "888" 
=================================================================================================================================================
=================================================================================================================================================
*/
void robotCom::handle_connect(const boost::system::error_code& error,tcp::resolver::iterator endpoint_iterator)
{
    if (!error)
    {

    }else if (endpoint_iterator != tcp::resolver::iterator())
    {
        socket_.close();
        tcp::endpoint endpoint = *endpoint_iterator;
        socket_.async_connect(endpoint, boost::bind(&robotCom::handle_connect, this,boost::asio::placeholders::error, ++endpoint_iterator));
    }
}