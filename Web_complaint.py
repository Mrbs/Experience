# !/usr/bin/env python
# coding=utf-8

import  Tkinter
from tkMessageBox import *
import	os,sys
import mysql.connector

def save_click():
    showinfo('温馨提示','感谢您的合作，我们尽快上门为您服务')
    cnx = mysql.connector.connect(user='root',password='root',host='192.168.1.129',database='HeatMeter')
    cur=cnx.cursor()
    cur.execute('insert into Complaint (id,ena,dis,tm) values (123,1,0,now())')
    cnx.close()


def edit_click():
    showinfo('温馨提示','感谢您的合作，我们继续提供更加优质服务')
    cnx = mysql.connector.connect(user='root',password='root',host='192.168.1.129',database='HeatMeter')
    cur=cnx.cursor()
    cur.execute('insert into Complaint (id,ena,dis,tm) values (123,0,1,now())')
    cnx.close()

root=Tkinter.Tk()


btn_Save=Tkinter.Button(root,
            anchor=Tkinter.E,       #指定对齐方式
            text='报修',           #指定按钮上的文本
            width=200,               #指定按钮宽度，相当于40个字符
            height=20, 
            bg='red',               #指定按钮高度，相当于5个字符
            command=save_click       #绑定事件
            )
btn_Save.pack()

btn_Edit=Tkinter.Button(root,
            anchor=Tkinter.E,       #指定对齐方式
            text='取消报修',           #指定按钮上的文本
            width=200,               #指定按钮宽度，相当于40个字符
            height=20,               #指定按钮高度，相当于5个字符
            bg='blue',              #指定按钮背景色
            command=edit_click      #绑定事件
        )
btn_Edit.pack()

root.mainloop()
