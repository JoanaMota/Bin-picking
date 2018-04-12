// char tppname[1024] = "TESTVERZ";
// char tppname[1024] = "TESTXZ";
// char tppname[1024] = "TESTYZ";
// char tppname[1024] = "TESTVERQ";
// char tppname[1024] = "TESTXQ";
// char tppname[1024] = "TESTYQ";
//char tppname[1024] = "TESTCZ";


//char tppname[1024] = "TESTROBOTHUMANOIDE";
char tppname[1024] = "TESTVISION";


/** THIS WORKS! **/
        robotCom c(io_service, iterator);
        c.runtpp(tppname);
        while(ros::ok())
        {
            ros::spinOnce();

            string value_cart = "";
            string value_joint = "";
            
            value_cart = c.getcrcpos();
            value_joint = c.getcrjpos();

            fanuc_cart.publish(value_cart);
            fanuc_joint.publish(value_joint);

            loop_rate.sleep();
        }
        c.motionstop();
        c.close();
        /* */


        //NOT WORKING 01 ????????? // c.movtocpos(Fanuc.Cnvert2Str("cart_all"));
        //NOT WORKING 03 ????????? // c.movtojpos(Temp1.Cnvert2Str("joint_all"));
        //NOT WORKING 10 LEITURA   // c.listtpp();
        //NOT WORKING 12 ?COMANDO? // c.motionstop();
        


        //WORKING     04 // c.getcrcpos();
        //WORKING     05 // c.getcrjpos();
        //WORKING     06 // c.checkcpos(Fanuc.Cnvert2Str("check_cart"));
        //WORKING     07 // c.checkjpos(Fanuc.Cnvert2Str("check_joint"));
        //WORKING     11 // c.runtpp(tppname);
