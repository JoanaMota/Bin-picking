#include "../include/fanuc_control/fanuc_control.h"

int check_for_input(int argc, char** argv)
{
    for(int i=0;i<argc;i++)
    {
        /** Check for -h or -help **/
        if( ( strcmp(argv[i],"-h") == 0) ||
            ( strcmp(argv[i], "-help") == 0)
          )
        {
            terminal_help();
            return -1;
        }

        /** Check for -d or -debug **/
        if( ( strcmp(argv[i],"-d") == 0) ||
            ( strcmp(argv[i], "-debug") == 0)
          )
            terminal_debug();

        /** Check for -s or -small **/
        if( ( strcmp(argv[i],"-s") == 0) ||
            ( strcmp(argv[i], "-small") == 0)
          )
            Global_IP = FANUC_SMALL;

        /** Check for -b or -big **/
        if( ( strcmp(argv[i],"-b") == 0) ||
            ( strcmp(argv[i], "-big") == 0)
          )
            Global_IP = FANUC_BIG;

        /** Check for -m or -manual **/
        if( ( strcmp(argv[i],"-m") == 0) ||
            ( strcmp(argv[i], "-manual") == 0)
          )
            manual_definition();

        /** Check for -d or -details **/
        if( ( strcmp(argv[i],"-dt") == 0) ||
            ( strcmp(argv[i], "-details") == 0)
          )
            detail_printing();


    }
}



void terminal_help(void)
{
    white_txt("==================================================================================\n");
    white_txt("==================================================================================\n");
    white_txt("                     ooooo   ooooo           oooo             \n                      888     888             888             \n                      888     888   .ooooo.   888  oo.ooooo.  \n                      888ooooo888  d88   88b  888   888   88b \n                      888     888  888ooo888  888   888   888 \n                      888     888  888    .o  888   888   888 \n                     o888o   o888o  Y8bod8P  o888o  888bod8P  \n                                                    888       \n                                                   o888o      \n");
    white_txt("==================================================================================\n");
    white_txt("==================================================================================\n");
    cout << endl;
    cout << endl;
    green_txt("**************\n");
    green_txt("*** INPUTS ***\n");
    green_txt("**************\n");
    green_txt("     -h  -help               ",1);    white_txt("shows the help menu (this one)\n",1);
    green_txt("     -b  -big                ",1);    white_txt("selects the big fanuc\n",1);
    green_txt("     -s  -small              ",1);    white_txt("selects the small fanuc\n",1);
    green_txt("     -m  -manual             ",1);    white_txt("select PORT and IP manually\n",1);
    green_txt("     -dt -details            ",1);    white_txt("prints the details selected by the user\n                             (this input should be the last one, so it'll print out the updated values)\n",1);
    green_txt("     -f  -file               ",1);    white_txt("selects the course from a file\n",1);
    green_txt("     -q  -question           ",1);    white_txt("allows the selection of the configuration using questions\n",1);
    green_txt("     -d  -debug              ",1);    white_txt("prints the debug in the terminal_help\n",1);
    cout << endl;
    cout << endl;
    yellow_txt("*****************   "); yellow_txt("The inputed arguments will be processed by the order of entry.\n",1);
    yellow_txt("*** ATTENTION ***   "); yellow_txt("For example, if you use -b -s, the node will assume the IP of the small fanuc\n",1);
    yellow_txt("*****************   "); yellow_txt("However, if you use -s -b, the node will assume the IP of the big fanuc\n",1);
    yellow_txt("Adittionaly, if you print the details -d and then change the IP, the details printed will not be updated!\n",1);
    cout << endl;
    cout << endl;
    white_txt("The ",1);
    yellow_txt("default");
    white_txt(" settings for this node are the following:\n",1);
    white_txt("     - small robot\n",1);
    white_txt("     - read path from another node\n",1);
    white_txt("     - no questions\n",1);
    white_txt("     - no details\n",1);
    white_txt("     - no debug\n",1);
    white_txt("     - no help\n",1);
    cout << endl;
    cout << endl;
    white_txt("After this menu, the node will shutdown!\n",1);
    white_txt("Good Luck\n");
    cout << endl;
    white_txt("For more information, please contact: ",1);
    white_txt("joao.peixoto@ua.pt");
    cout << endl;
    white_txt("==================================================================================\n");
    white_txt("==================================================================================\n");
    return;
}



void terminal_debug(void)
{
    /*
QUANDO TODO O CODIGO ESTIVER OPERACIONAL, RETIRAR ESTE COMENTARIO
(SERVE APENAS PARA NAO TER QUE RESPONDER SEMPRE AS PERGUNTAS)!!!


    string answer1, answer2, answer3, answer4, asnwer5;

    // Question 1: global_debug_show_functions //
    cout << endl;;
    do{ cout << "\033[F\rDo you wish to see the name of the functions and their description? [y/n]: ";
        cin >> answer1;} while(answer1 != "y" && answer1 != "n");

    // Question 2: global_debug_stop_functions //
    cout << endl;;
    do{ cout << "\033[F\rDo you wish to stop at the beggining of a function? [y/n]: ";
        cin >> answer2;} while(answer2 != "y" && answer2 != "n");
    // Question 3: global_debug_print_structs //
    cout << endl;;
    do{ cout << "\033[F\rDo you wish to print the struct values? [y/n]: ";
        cin >> answer3;} while(answer3 != "y" && answer3 != "n");
    // Question 4: global_debug_print_input_vars //
    cout << endl;;
    do{ cout << "\033[F\rDo you wish to see the input values of a function? [y/n]: ";
        cin >> answer4;} while(answer4 != "y" && answer4 != "n");
    // Question 4: Global_debug_show_results //
    cout << endl;;
    do{ cout << "\033[F\rDo you wish to see the output values of a function? [y/n]: ";
        cin >> answer5;} while(answer5 != "y" && answer5 != "n");


    // Global variable setting //
    if(answer1 == "y")
        Global_debug_show_functions = true;
    if(answer2 == "y")
        Global_debug_stop_functions = true;
    if(answer3 == "y")
        Global_debug_print_structs = true;
    if(answer4 == "y")
        Global_debug_print_input_vars = true;
    if(answer5 == "y")
        Global_debug_show_results = true;
*/
        Global_debug_show_functions = true;
        Global_debug_stop_functions = true;
        Global_debug_print_structs = true;
        Global_debug_print_input_vars = true;
        Global_debug_show_results = true;


    return;
}



void manual_definition(void)
{
    string IP_temp;
    string Port_temp;
    string answer1, answer2;
    bool correct1 = false, correct2 = false;
    
    // Sets the IP //
    while(!correct1)
    {
        cout << "Set the IP XXX.XXX.X.XXX: ";
        cin  >> IP_temp;
        cout << endl;
        do{ cout << "\033[F\rIs this '" << IP_temp << "' the correct IP? [y/n]: ";
            cin >> answer1;} while(answer1 != "y" && answer1 != "n");
        if(answer1 == "y")
            correct1 = !correct1;
    }

    // Sets the Port//
    while(!correct2)
    {
        cout << "Set the Port: ";
        cin  >> Port_temp;
        cout << endl;
        do{ cout << "\033[F\rIs this '" << Port_temp << "' the correct Port? [y/n]: ";
            cin >> answer2;} while(answer2 != "y" && answer2 != "n");
        if(answer2 == "y")
            correct2 = !correct2;
    }

    Global_IP   = IP_temp;
    Global_Port = Port_temp;
}



void detail_printing(void)
{
    green_txt("********************************\n");
    green_txt("*** IP Adress: "); green_txt(Global_IP,1);   green_txt(" ***\n"); 
    green_txt("*** Port:      "); green_txt(Global_Port,1); green_txt("          ***\n");
    green_txt("********************************\n\n\n");
    return;
}