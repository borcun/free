#!/usr/bin/python3.5
from tkinter import *

def check_login( username, password ):
    if username == 'tkinter' and password == '123456':
        state_label.config( text="Succeed", fg="green" )
    else:
        state_label.config( text= "Failed", fg="red" )
        
def return_callback( event ):
    check_login( user_entry.get(), pass_entry.get() )

def close_root( event ):
    root.destroy()

root = Tk()
root.title( "Login" )
root.bind( "<Escape>", close_root )

login_frame = Frame( root )
user_label = Label( login_frame, text="Username: ", font="bold 10" )
user_entry = Entry( login_frame )
pass_label = Label( login_frame, text="Password: ", font="bold 10" )
pass_entry = Entry( login_frame )
state_label = Label( login_frame )

user_entry.focus()
user_entry.bind( "<Return>", return_callback )
pass_entry.config( show="*")
pass_entry.bind( "<Return>", return_callback )

login_button = Button( login_frame, text="Login", command=lambda: check_login( user_entry.get(), pass_entry.get() ) )

user_label.grid( row=0, column=0 )
user_entry.grid( row=0, column=1 )
pass_label.grid( row=1, column=0 )
pass_entry.grid( row=1, column=1 )
login_button.grid( row=2, column=1, sticky=E )  
state_label.grid( row=2, column=0, sticky=W )

login_frame.pack()

root.mainloop()
