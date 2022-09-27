import scapy.all as scapy
import socket
import easygui as eg

def scan(ip):
    arp_req_frame = scapy.ARP(pdst = ip)

    broadcast_ether_frame = scapy.Ether(dst = "ff:ff:ff:ff:ff:ff")
    
    broadcast_ether_arp_req_frame = broadcast_ether_frame / arp_req_frame

    answered_list = scapy.srp(broadcast_ether_arp_req_frame, timeout = 1, verbose = False)[0]
    result = []
    for i in range(0,len(answered_list)):
        client_dict = {"ip" : answered_list[i][1].psrc, 
                       "mac" : answered_list[i][1].hwsrc}
        result.append(client_dict)

    return result
  
def display_result(result):
    print("-----------------------------------\nIP Address\tMAC Address\n-----------------------------------")
    for i in result:
        print("{}\t{}".format(i["ip"], i["mac"]))

def getExternalIPs():

    scanned_output = scan('192.168.1.1/36')

    externalIPs = scanned_output
    # display_result(scanned_output)

    print(externalIPs)

    # Filtering out computer IP from list
    tempfilter = list(filter(lambda i: i['ip'] != socket.gethostbyname(socket.gethostname()), externalIPs))

    # Filtering out possible router IP from list
    externalIPs = list(filter(lambda i: not((i['ip']).endswith('.1')), tempfilter))

    if len(externalIPs) >= 2:
        externalIPs = UserExcudedIPs(externalIPs)

    return externalIPs

def UserExcudedIPs(listOfOptions):

    question = "Check to remove any non-ESP IPs"
    title = "Manuel IP address removal"
    choice = eg.multchoicebox(question , title, listOfOptions, [])
    ChoiceRemoved = listOfOptions

    for j in range(len(choice)):
        ChoiceRemoved = list(filter(lambda i: i['ip'] != ast.literal_eval(choice[j])['ip'], ChoiceRemoved))
    
    return ChoiceRemoved


if __name__ == "__main__":
    externalIPs = getExternalIPs()
    display_result(externalIPs)
    