#!/usr/bin/env python3
import subprocess

class Connectivity:
    def __init__(self, ip, count=2):
        self.ip = str(ip)
        self.ping_count = str(count)

    def wifiping(self):
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

        print("\nDictionary: ", ping_values)


gh = Connectivity("192.168.1.1",10)
# gh = Connectivity("localhost")
# gh = Connectivity("sdgdgdhdf")

gh.wifiping()