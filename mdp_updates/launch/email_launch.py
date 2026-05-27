
# run from mdp-packages

import smtplib
from getpass import getpass
from mdp_updates.loop.email_loop import email_loop

SMTP_SERVER = "smtp.gmail.com"
SMTP_PORT = 465

sender_email = "christianplevier@gmail.com"
app_password = "anwz mhre dmdx yznt"

recipient_email = input("please enter the email to reach here: ")

email_loop(sender_email,app_password,recipient_email)