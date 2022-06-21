1、模型可以选择自己训练的模型，270nice.pt训练出来的效果仅供参考
2、所有文件必须放在一个文件夹下
3、light.py为单独检测红绿灯的程序
4、DL_dete.py为单独检测车道线的程序，在此之前，请先运行qingzhou_brigup.launch
5、运行整个代码，请按照如下顺序启动（防止开机第一次运行就卡死），
     roslaunch qingzhou_bringup qingzhou_bringup.launch
     roslaunch qingzhou_nav l1_nav_pure.launch    //导航
     python DL_control.py   //发点程序
     python got_it_socket.py    //和笔记本建立连接 socket
     python3 DL_final_dete.py  //视觉（红绿灯和S弯）
6、在x64架构的电脑下运行ChatServer，这是上位机