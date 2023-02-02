from selenium import webdriver
import time
import sys
import yaml
from yaml.loader import SafeLoader

with open("password.yml") as f:
        conf = yaml.load(f, Loader=SafeLoader)
        minerva_email = conf['minerva_user']['email']
        minerva_pass = conf['minerva_user']['password']
        # CRNS
        crn = []
        crn.append(conf['CRN']["lecture"])
        crn.append(conf['CRN']["lab_t"])
        crn.append(conf['CRN']["lab_w"])
    
print(crn)
            