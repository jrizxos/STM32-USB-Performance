#include <iostream>
#include <iomanip>
#include <time.h>
#include <libusb-1.0/libusb.h>
using namespace std;

void printdev(libusb_device* dev);
int recieve(libusb_device_handle* dev_handle, int msg_length, int msg_limit);
int readprint(libusb_device_handle* dev_handle, int msg_length, int msg_limit);
int readmeasure(libusb_device_handle* dev_handle, int msg_length, int msg_limit);
int cobsdecode(unsigned char* buffer, int length, unsigned char* data);

int main() {
	libusb_context *ctx = NULL;		//a libusb session, NULL is all we need
	libusb_device_handle* dev_handle;	//a device handle
	libusb_device* dev;					//pointer to device, used to retrieve info about the device

	int r;								//for return values

	//initialize a library session
	r = libusb_init(NULL);				
	if(r < 0) {
		cout << "Initialization Error: : " << libusb_error_name(r) << endl;
				return 1;
	}

	//set verbosity level, as suggested in the documentation
	r = libusb_set_option(NULL,			
		LIBUSB_OPTION_LOG_LEVEL, LIBUSB_LOG_LEVEL_WARNING); 
	if (r != LIBUSB_SUCCESS) {
		cout << "Option Error : " << libusb_error_name(r) << endl;
		return 1;
	}

	//open our device
	dev_handle = libusb_open_device_with_vid_pid(NULL, 1155u, 22336u); //0x0483:0x5740
	if (dev_handle == NULL) {
		cout << "Cannot open device!" << endl;
		return 1;
	}
	else cout << "Device Opened:" << endl;

	//print info
	dev = libusb_get_device(dev_handle);
	libusb_ref_device(dev);
	printdev(dev);
	libusb_unref_device(dev);

	//find out if kernel driver is attached
	if (libusb_kernel_driver_active(dev_handle, 0) == 1) {
		cout << "Kernel Driver Active!" << endl;
		//detach it if it is
		if (libusb_detach_kernel_driver(dev_handle, 0) == 0)
			cout << "Kernel Driver Detached!" << endl;
	}

	//claim interface 0 (required)
	r = libusb_claim_interface(dev_handle, 0);
	if (r < 0) {
		cout << "Cannot Claim Interface 0 : " << libusb_error_name(r) << endl;
		return 1;
	}
	cout << "Claimed Interface 0." << endl;
	//claim interface 1 (the one we want)
	r = libusb_claim_interface(dev_handle, 1); 
	if (r < 0) {
		cout << "Cannot Claim Interface 1 : " << libusb_error_name(r) << endl;
		return 1;
	}
	cout << "Claimed Interface 1." << endl;

	// read data using only one of the following methods
	//r = readprint(dev_handle, 1023, 2);   // reads an prints every message until limit, be careful with printing long messages! (suitable for continous transmit)
	//r = readmeasure(dev_handle, 1023, 2);	// reads every message until limit then calculates and prints performance stats (suitable for continous transmit)
	r = recieve(dev_handle, 50000, 2);		// receives messages that contain COBS encoded packets (suitable for continous transmit optimized)
	
	//release the claimed interfaces, and close the device
	libusb_release_interface(dev_handle, 0); 
	libusb_release_interface(dev_handle, 1); 
	libusb_close(dev_handle);			//close the device we opened
	libusb_exit(NULL);					//close the session
	return r;
}

/**
  * @brief Receives messages containing COBS encoded packets, prints performance stats
  * @param dev_handle: pointer to device
  * @param msg_length: max message length to read
  * @param msg_limit: maximum ammount of messages to read
  * @retval 0 is OK, >0 is error
  */
int recieve(libusb_device_handle* dev_handle, int msg_length, int msg_limit){
	int actual, r;											//read byte counter
	time_t start_time, end_time;							//time counters
	double timer;
	int msg_count = 0;										//number of messages we recieved
	int empty_packets = 0;									//number of empty packets
	const unsigned int timeout = 100000u;					//max time to read a message (ms)
	const unsigned char endpoint = 129u;					//endpoint address to read from
	unsigned char* data = new unsigned char[msg_length];	//buffer to store read data 

	int array_idx = 0;
	unsigned char* data_array = new unsigned char[msg_length*msg_limit];
	
	cout << "Recieving Data..." << endl << endl;
	time(&start_time);
	//read loop
	while(msg_count < msg_limit){
		//read data, transfer direction is decided by endpoint properties, here it's an IN endpoint
		r = libusb_bulk_transfer(dev_handle, endpoint, data, msg_length, &actual, timeout);
		//if it is a valid message store it
		if(r == 0){
			cout << '\r' << std::setw(6) << std::setfill('0') << msg_count << std::flush;
			for(int i = 0; i < actual; i++, array_idx++){
				data_array[array_idx] = data[i];
			}
			msg_count++;
		}
		//else print error and stop reading
		else{
			//discard empty packets, those are not an error
			if(actual == 0) {
				empty_packets++;
				continue;
			}
			cout << "Read Error : " << libusb_error_name(r) << ", actual bytes read : " << actual << endl;
			return 1;
		}
	}
	time(&end_time);

	// decode COBS and count received bytes
	int size = 0;
	int prev = 0;
	int total = 0;
	unsigned char* temp = new unsigned char[msg_length];
	for(int i = 0; i < array_idx; i++){
		if(data_array[i] == 0){
			actual = cobsdecode((unsigned char*)data_array + prev, size, temp);
			total += actual;
			prev = i + 1;
			size = 0;
		}
		else size++;
	}

	// print performance stats
	timer = difftime(end_time, start_time);
	cout << endl << "Transmision ended." << endl;
	cout << "Time : " << timer << " seconds." << endl;
	cout << "Total data : " << total << " bytes." << endl;
	cout << "(Empty packets : " << empty_packets << ")" << endl;
	cout << "--------------------------------------------------" << endl;
	cout << "Bandwidth : " << (double)total / timer << " bytes/sec." << endl;
	cout << "--------------------------------------------------" << endl;

	return 0;
}

/**
  * @brief Decodes a COBS encoded packet
  * @param buffer: input buffer
  * @param length: length of data in input buffer
  * @param data: output buffer to store decoded data
  * @retval number of bytes decoded 
  */
int cobsdecode(unsigned char* buffer, int length, unsigned char* data){
	const uint8_t* byte = buffer;		// Encoded input byte pointer
	uint8_t* decode = (uint8_t*)data;	// Decoded output byte pointer

	for(uint8_t code = 0xff, block = 0; byte < buffer + length; --block){
		if(block)						// Decode block byte
			*decode++ = *byte++;
		else{
			if(code != 0xff)			// Encoded zero, write it
				*decode++ = 0;
			block = code = *byte++;		// Next block length
			if(!code)					// Delimiter code found
				break;
		}
	}

	return (int)(decode - (uint8_t*)data);
}


/**
  * @brief Reads messages and prints them as base 10 bytes, be careful with printing long messages!
  * @param dev_handle: pointer to device
  * @param msg_length: max message length to read
  * @param msg_limit: maximum ammount of messages to read
  * @retval 0 is OK, >0 is error
  */
int readprint(libusb_device_handle* dev_handle, int msg_length, int msg_limit){
	int actual, r;											//read byte counter
	int msg_count = 0;										//number of messages we recieved
	int empty_packets = 0;									//number of empty packets (when packet length = 64 one empty is send after)
	const unsigned int timeout = 10000u;					//max time to read a message (ms)
	const unsigned char endpoint = 129u;					//endpoint address to read from
	unsigned char* data = new unsigned char[msg_length];	//data to read buffer 

	cout << "Reading Data..." << endl << endl;
	//read loop
	while (msg_count < msg_limit){
		//read data, transfer direction is decided by endpoint properties, here it's an IN endpoint
		r = libusb_bulk_transfer(dev_handle, endpoint, data, msg_length, &actual, timeout);
		//if it is a valid message print it
		if (r == 0 && actual == msg_length){
			cout << msg_count << "("<< actual << "): " << endl;
			for(int i = 0; i < msg_length; i++){
				cout << (int)data[i] << " ";
			}
			msg_count++;
			cout << endl << endl;
		}
		//else print error and stop reading
		else{
			//discard empty packets, those are not an error
			if (actual == 0) {
				empty_packets++;
				continue;
			}
			cout << "Read Error : " << libusb_error_name(r) << ", actual bytes read : " << actual << endl;
			return 1;
		}
	}
	cout << empty_packets << " empty packets." << endl;
	return 0;
}


/**
  * @brief Reads messages and prints performance stats
  * @param dev_handle: pointer to device
  * @param msg_length: max message length to read
  * @param msg_limit: maximum ammount of messages to read
  * @retval 0 is OK, >0 is error
  */
int readmeasure(libusb_device_handle* dev_handle, int msg_length, int msg_limit){
	int actual, r;												//read byte counter
	time_t start_time, end_time;								//time counters
	double timer;
	int msg_count = 0;											//number of messages we recieved
	int empty_packets = 0;										//number of empty packets
	const unsigned int timeout = 10000u;						//max time to read a message (ms)
	const unsigned char endpoint = 129u;						//endpoint address to read from
	unsigned char* data = new unsigned char[msg_length];		//data to read buffer 
	//allocate memory to keep all read packets so we can check them after the transmission
	unsigned char** data_array = new unsigned char*[msg_limit];
	for (int i = 0; i < msg_limit; i++)
		data_array[i] = new unsigned char[msg_length];
	
	cout << "Reading Data..." << endl ;
	time(&start_time);
	//read loop
	while (msg_count < msg_limit) {
		//read data, transfer direction is decided by endpoint properties, here it's an IN endpoint
		r = libusb_bulk_transfer(dev_handle, endpoint, data, msg_length, &actual, timeout);
		//if it is a valid message add it to the array
		if (r == 0 && actual == msg_length) {
			cout << '\r' << std::setw(6) << std::setfill('0') << msg_count << std::flush;
			/*for(int i = 0; i < msg_length; i++) {
				data_array[msg_count][i] = data[i];
			}*/
			msg_count++;
		}
		//else print error and stop reading
		else {
			//discard empty packets, those are not an error
			if (actual == 0) {
				empty_packets++;
				continue;
			}
			cout << "Read Error : " << libusb_error_name(r) << ", actual bytes read : " << actual << endl;
			return 1;
		}
	}

	// print performance stats
	time(&end_time);
	timer = difftime(end_time, start_time);
	cout << endl << "Transmision ended." << endl;
	cout << "Time : " << timer << " seconds." << endl;
	cout << "Packet length : " << msg_length << endl;
	cout << "Packets read : " << msg_count << endl;
	cout << "(Empty packets : " << empty_packets << ")" << endl;
	cout << "--------------------------------------------------" << endl;
	cout << "Bandwidth : " << (msg_limit * (double)msg_length) / timer << " bytes/sec." << endl;
	cout << "--------------------------------------------------" << endl;

	return 0;
}

/**
  * @brief Prints info about the device
  * @param dev: pointer to device
  * @retval None
  */
void printdev(libusb_device *dev) {
	libusb_device_descriptor desc;
	int r = libusb_get_device_descriptor(dev, &desc);
	if (r < 0) {
		cout<<"failed to get device descriptor"<<endl;
		return;
	}
	cout<<"Number of possible configurations: "<<(int)desc.bNumConfigurations<<"  ";
	cout<<"Device Class: "<<(int)desc.bDeviceClass<<"  ";
	cout<<"VendorID: "<<desc.idVendor<<"  ";
	cout<<"ProductID: "<<desc.idProduct<<endl;
	libusb_config_descriptor *config;
	libusb_get_config_descriptor(dev, 0, &config);
	cout<<"Interfaces: "<<(int)config->bNumInterfaces << endl;
	const libusb_interface *inter;
	const libusb_interface_descriptor *interdesc;
	const libusb_endpoint_descriptor *epdesc;
	for(int i=0; i<(int)config->bNumInterfaces; i++) {
		inter = &config->interface[i];
		cout << "Number of alternate settings: " << inter->num_altsetting << endl;
		for(int j=0; j<inter->num_altsetting; j++) {
			interdesc = &inter->altsetting[j];
			cout<<"\tInterface Number: "<<(int)interdesc->bInterfaceNumber<<" | ";
			cout<<"Number of endpoints: "<<(int)interdesc->bNumEndpoints<< endl;
			for(int k=0; k<(int)interdesc->bNumEndpoints; k++) {
				epdesc = &interdesc->endpoint[k];
				cout<<"\t\tDescriptor Type: "<<(int)epdesc->bDescriptorType<<" | ";
				cout<<"EP Address: "<<(int)epdesc->bEndpointAddress<< endl;
			}
		}
		cout << endl;
	}
	cout<<endl<<endl;
	libusb_free_config_descriptor(config);
}
