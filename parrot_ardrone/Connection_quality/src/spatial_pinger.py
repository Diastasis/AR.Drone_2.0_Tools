#!/usr/bin/env python3
import subprocess

class Connectivity:
    def __init__(self, ip, count=3):
        self.ip = str(ip)
        self.ping_count = str(count)

    def ping(self):
        ping_values = {}
        args = ["ping","-c", self.ping_count, self.ip]
        # subprocess allow the execution of linux commands and it captures the output
        output = subprocess.Popen( args, stdout=subprocess.PIPE,stderr=subprocess.PIPE ).communicate()

        if "Name or service not known" in str(output[1]):
            print("Invalid IP address")
            return
        elif "Network is unreachable" in str(output[1]):
            print("Network is unreachable")
            return
        else:
            output = str(output[0]).split("--- " + self.ip + " ping statistics ---")[1].replace("'","").replace("ms","").replace("%","").strip("\\n").split("\\n")
            # print(output)
        for word in output[0].split(","):
            word1, word2 = word.strip().split(" ",1)
            if word1.isnumeric():
                ping_values[word2] = int(word1)
            else:
                ping_values[word1] = int(word2)
        if len(output) < 2:
            print("\nDictionary: ", ping_values)
            return

        names = output[1].replace("ms","").replace("rtt","").replace(" ","").strip(" ").split("=")[0].split("/")
        values = output[1].replace("ms","").replace("rtt","").replace(" ","").strip(" ").split("=")[1].split("/")
        for i, name in enumerate(names):
            ping_values[name] = float(values[i])

        # print("\nDictionary: ", ping_values)
        return ping_values

width = 3
w_step = 1

length = 3
l_step = 1

height = 1
h_step = 0.5

# gh = Connectivity("192.168.1.1",10)
# gh = Connectivity("172.18.29.1",10)
gh = Connectivity("localhost",2)

cc = 0
exportable = {}
for h in range(int((height-h_step)/h_step)):
    for w in range(int((width-w_step)/w_step)):
        for l in range(int((length-l_step)/l_step)):

            try:
                print("\nNext point:",(w,l,h),"[Enter]")
                input("")
            except SyntaxError:
                pass
            
            temp = gh.ping()
            exportable[(w,l,h)] = temp
            cc+=1
print(exportable)

print("Total points:", cc)