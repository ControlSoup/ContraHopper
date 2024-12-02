import re
import yaml
import os
import argparse 

'''
Purpose of this file:
    - Ingests a yaml config for udp ouputs
    - Constructs a file that will send, receive and map network outputs channels
'''

__FOOTER = '''// Setup function to call during intializaiton
void setup_udp(){
    int status = WL_IDLE_STATUS;
    Serial.println("INFO: Setting up udp connection");
    WiFi.begin(SSID, PASSWORD);

    // Wait for connection
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("INFO: Connected to wifi");
    Udp.begin(LOCAL_PORT);
    Serial.printf("INFO: Now listening at IP %s, UDP port %d\\n", WiFi.localIP().toString().c_str(), LOCAL_PORT);

    // Get the ip to send to  
    char incomingPacket[255];  
    bool readPacket = false;
    while (!readPacket) {
        Serial.println("INFO: Waiting on packet from you!");
        int packetSize = Udp.parsePacket();
        if (packetSize){

            Serial.printf(
                "INFO: Received %d bytes from %s, port %d\\n", 
                packetSize, 
                Udp.remoteIP().toString().c_str(), 
                Udp.remotePort()
            );

            int len = Udp.read(incomingPacket, 255);
            if (len > 0){
                incomingPacket[len] = 0;
            }

            Serial.printf("INFO: Recieved UDP packet contents: %s\\n", incomingPacket);
            readPacket = true;
        } 
    }

    // Send the header over a couple times to make sure they got it (if not they can just decode)
    int i;
    for (i = 0; i < 3; i++){
        Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
        Udp.print(NWO_HEADER);
        Udp.endPacket();
        delay(500);
    }

}

NWI recieve_udp(){
    // Messages come in 3 chars
    static char incomingPacket[4] = {105, 105, 105, 0}; 

    int packetSize = Udp.parsePacket();
    if (packetSize){

        int len = Udp.read(incomingPacket, 3);

        // Null terminate length
        if (len > 0){
            incomingPacket[len] = 0;
        } 

    } 

    return hash_input(incomingPacket);
}
'''


# Ingests a yaml file and builds the "network output information", and udp writing info
def ouput_files(yaml_path: str, password: str, output_path: str):

    def parse_yaml(file_path: str):
        parsed = {}
        with open(file_path, 'r') as stream:
            parsed = yaml.safe_load(stream)

        if len(set(parsed['Outputs'])) != len(parsed['Outputs']):
            raise ValueError(f"Config file has duplicate Output Keys: \n{parsed['Outputs']}") 

        return parsed['Network'], parsed['Outputs'], parsed['Inputs'] 

    def add_line(file: list, line: str):
        file.append(line + '\n')

    def add_line_tabbed(file: list, line: str, tab_level = 1):
        file.append('\t' * tab_level)
        add_line(file, line)

    def empty_lines(file: str, num):
        file.append(num * '\n')

    def get_brack_units(string: str):
        return re.findall(r"\[(.*?)\]", string)[0] 

    def underscore_key(key: str):
        new_key = key
        if ' [' in key:
            units = get_brack_units(key)
            units = units.replace('/', 'p')
            units = units.replace('*', '')
            units = units.replace('-', '')
            units = units.replace('%', 'percent')
            key = key.replace('-', '_')
            new_key = key.split(' [')[0] + f'__{units}' 

        return new_key
        

    # Get yaml info 
    network_info, outputs, inputs = parse_yaml(yaml_path)

    # Init a file
    file = []

    # Headers
    add_line(file, "#include <WebServer.h>")
    add_line(file, "#include <WiFi.h>")
    add_line(file, "#include <WiFiUdp.h>")
    empty_lines(file, 2)

    # Udp network class (Ardunio)
    add_line(file, 'WiFiUDP Udp;')
    empty_lines(file, 2)

    # Network settings
    add_line(file, '// Network Information')
    add_line(file, f'const char* SSID = "{network_info["SSID"]}";')
    add_line(file, f'const char* PASSWORD = "{password}";')
    add_line(file, f'const unsigned int LOCAL_PORT = {network_info["port"]};')
    empty_lines(file, 2)


    # Send UDP Function
    add_line(file, '// Send function to pass data to connection')
    add_line(file, 'void send_udp(){')
    add_line_tabbed(file, 'Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());')

    for key in outputs[:-1]:
        add_line_tabbed(file, f'Udp.printf("%-10f,", NWO_{underscore_key(key)});')
    add_line_tabbed(file, f'Udp.printf("%-10f", NWO_{underscore_key(outputs[-1])});')
    add_line_tabbed(file, f'Udp.endPacket();')
    add_line(file, '}')
    empty_lines(file, 2)



    header = []

    # Header file for globals
    add_line(header, '// Network Outputs')
    add_line(header, 'const char* NWO_HEADER = (')
    for key in outputs[:-1]:
        add_line(header, f'\t"{key},"')
    add_line(header, f'\t"{outputs[-1]}"')
    add_line(header, ');')

    for key in outputs:
        add_line(header, f'float NWO_{underscore_key(key)} = -404.0;')
    empty_lines(header, 2)


    # Simple Enum Hash for incoming messages

    add_line(header, '// Network Inputs')
    add_line(header, 'enum NWI{')
    for pairing in inputs: 
        key = next(iter(pairing))
        val = pairing[key]
        add_line_tabbed(header, f'NWI_{key},')
    add_line_tabbed(header, 'NWI_UNKNOWN')
    add_line(header, '};')
    empty_lines(header, 2)

    add_line(file, 'NWI hash_input(char input[4]){')
    add_line_tabbed(file, f'if (strcmp(input, "{inputs[0][next(iter(inputs[0]))]}") == 0){{return NWI_{next(iter(inputs[0]))};}}')
    for pairing in inputs[1:]:
        key = next(iter(pairing))
        val = pairing[key]
        add_line_tabbed(file, f'else if (strcmp(input, "{val}") == 0) {{return NWI_{key};}}')
    add_line_tabbed(file, 'else {return NWI_UNKNOWN;}')
    add_line(file, '};')
    empty_lines(file, 2)


    # Add remaining functions
    add_line(file, __FOOTER)



    # Write files
    def write_file(path: str, file_name: str, contents: list[str]):
        if not os.path.exists(path): 
            os.makedirs(path)

        file_path = os.path.join(path, file_name)
        try:
            os.remove(file_path)
        except FileNotFoundError:
            pass

        with open(file_path, 'w') as f:
            for line in contents:
                f.write(line)

    write_file(output_path, 'network_outputs.h', header)
    write_file(output_path, 'network_outputs.ino', file)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Build UDP Data recording file')

    # Add arguments
    parser.add_argument('-i', '--input', help='Input file path', required=True)
    parser.add_argument('-p', '--password', help='Network Password', required=True)

    # Parse the arguments
    args = parser.parse_args()

    # Write the files 
    ouput_files(args.input, args.password, 'Asoftware/')
