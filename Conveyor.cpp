// Huỳnh Tuấn Kiệt - 2010364
#include <stdio.h>
#include <winsock.h>
#include <winioctl.h>
#define TCP_PRT 0
#define UDP_PRT 1
#define DATA_SIZE 4096
int sd; // Socket Discripter
struct sockaddr_in my;
struct sockaddr_in dst;
struct sockaddr_in from;
short DATAi; // Number of data items to send
short MDATAi; // MEMOBUS data length
unsigned char iSerial; // Serial number

// Protocol declaration
int memobus_master_main( unsigned short trans_prt, unsigned long myip,
unsigned short myport, unsigned long dstip, unsigned short dstport );
int memobus_msg( unsigned short trans_prt, char∗ sbuf, char∗ rbuf );
int tcp_msg( char∗ sbuf, char∗ rbuf );
int udp_msg( char∗ sbuf, char∗ rbuf );
void mk_cmd_data( unsigned char SFC, unsigned char CPUNum,
unsigned short Adr, unsigned short DataNum, char∗ sbuf );
int chk_rsp_data( int rlen, char∗ sbuf, char∗ rbuf );

/∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗/
/∗ Open a TCP/UDP port ∗/
/∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗/
int memobus_master_main( unsigned short trans_prt,
unsigned long myip, unsigned short myport,
unsigned long dstip, unsigned short dstport )
{
    WSADATA wsadata;
    int rc;
    // Declaration to use Winsock.dll (must be declared before calling routines)
    rc = WSAStartup( 0x0101, &wsadata );
    if ( rc != 0 )
    {
        exit(0);
    }
    // Initialize serial number setting
    iSerial = 0x00;
    // Clear the sockaddr structure (IP address, port number, etc.) with zeros.
    memset( (char ∗)&my, 0, sizeof(struct sockaddr));
    memset( (char ∗)&dst, 0, sizeof(struct sockaddr));
    // Declare the PC’s IP address and port number
    my.sin_family = AF_INET;
    my.sin_addr.s_addr = myip;
    my.sin_port = htons( myport );
    // Declare the MP3000’s IP address and port number
    dst.sin_family = AF_INET;
    dst.sin_addr.s_addr = dstip;
    dst.sin_port = htons( dstport );
    // Create the TCP or UDP socket
    if( trans_prt == TCP_PRT )
    { //TCP
        sd = socket( AF_INET, SOCK_STREAM, 0 );
    }
    else
    { //UDP
        sd = socket( AF_INET, SOCK_DGRAM, 0 );
    }
    if ( sd <= 0 )
    {
        rc = -1;
        return( rc );
    }
    // Bind the local port number and socket
    rc = bind( sd, ( struct sockaddr ∗)&my, sizeof(struct sockaddr_in));
    if ( rc == -1 )
    {
        closesocket( sd );
        rc = -2;
        return( rc );
    }
    // Establish connection
    if( trans_prt == TCP_PRT )
    { //TCP
        rc = connect( sd, ( struct sockaddr ∗)&dst, sizeof(struct sockaddr_in));
        if( rc == -1 )
        { //TCP
            closesocket( sd );
            rc = -3;
            return( rc );
        }
    }
    return(rc);
}

/
∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗/
/∗ Send command data, receive response data ∗/
/∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗/
int memobus_msg( unsigned short trans_prt, char∗ sbuf, char∗ rbuf )
{
    int rc;
    if( trans_prt == TCP_PRT)
    {
        rc = tcp_msg(sbuf, rbuf);
    }
    else
    {
        rc = udp_msg(sbuf, rbuf);
    }
    return(rc);
}

/
∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗/
/∗ Send command data, receive response data (TCP) ∗/
/∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗/
int tcp_msg( char∗ sbuf, char∗ rbuf )
{
    int slen, rlen;
    int rc = 0;
    // Send command data
    slen = send( sd, sbuf, DATAi, 0 );
    if ( slen != DATAi ) // Returns the number of bytes sent if sending was successful
    {
        closesocket(sd);
        rc = -1;
        return ( rc );
    }
    // Receive response data
    rlen = recv( sd, rbuf, DATA_SIZE, 0 );
    if ( rlen <= 0 ) //A 0 or less value is returned if receiving failed
    {
        closesocket(sd);
        rc = -2;
        return ( rc );
    }
    // Check response data
    rc = chk_rsp_data( rlen, sbuf, rbuf );
    if ( rc != 0 ) //Error in received data
    {
        closesocket(sd);
        return ( rc );
    }
    iSerial++; // Increment the serial number of the 218 header
    return ( rc );
}

/
∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗/
/∗ Send command data, receive response data (UDP) ∗/
/∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗/
int udp_msg( char∗ sbuf, char∗ rbuf )
{
    int slen, rlen, fromlen;
    int rc = 0;
    // Send command data
    slen = sendto( sd, sbuf, DATAi, 0, (struct sockaddr ∗)&dst, sizeof(struct sockaddr));
    if ( slen != DATAi ) // Returns the number of bytes sent if sending was successful
    {
        closesocket(sd);
        rc = -1;
        return ( rc );
    }
    // Receive response data
    fromlen = sizeof(struct sockaddr);
    rlen = recvfrom( sd, rbuf, DATA_SIZE, 0, (struct sockaddr ∗)&from, &fromlen );
    if ( rlen <= 0 ) //A 0 or less value is returned if receiving failed
    {
        closesocket(sd);
        rc = -2;
        return ( rc );
    }
    // Check response data
    rc = chk_rsp_data( rlen, sbuf, rbuf );
    if ( rc != 0 ) //Error in received data
    {
        closesocket(sd);
        return ( rc );
    }
    iSerial++; // Increment the serial number of the 218 header
    return ( rc );
}

/
∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗/
/∗ Create Extended MEMOBUS protocol command ∗/
/∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗/
void mk_cmd_data( unsigned char SFC, unsigned char CPUNum,
unsigned short Adr, unsigned short DataNum, char∗ sbuf )
{
    //Calculate number of data items
    //MEMOBUS data length from MFC to end of data
    switch(SFC)
    {
        case 0x09:
            MDATAi = 8;
            break;
        default:
            break;
    }
    //Calculate total number of data items
    DATAi = MDATAi +14; //218 header (12 bytes) + Length part (2 bytes) + MEMOBUS data length
    (variable)
    // Create the 218 header part
    // Initialize the send/receive buffers
    memset( sbuf, 0x00, DATA_SIZE );
    // Set the register type.
    sbuf[0] = 0x11; // Extended MEMOBUS command
    // Set the serial number and increment for every transmission
    sbuf[1] = iSerial;
    // Set the destination channel number
    sbuf[2] = 0x00; // Always 0 hex because the PLC channel is undefined
    // Set the source channel number
    sbuf[3] = 0x00; // Always 0 hex because channel numbers do not apply to PCs
    sbuf[4] = 0x00; // Reserved.
    sbuf[5] = 0x00; // Reserved.
    // Set the total number of data items from the start of the 218 header to the end of MEMOBUS data
    sbuf[6] = (unsigned char)(DATAi & 0x00FF); // Data length (L)
    sbuf[7] = (unsigned char)((DATAi & 0xFF00) >> 8); // Data length (H)
    sbuf[8] = 0x00; // Reserved.
    sbuf[9] = 0x00; // Reserved.
    sbuf[10] = 0x00; // Reserved.
    sbuf[11] = 0x00; // Reserved.
    // Create the MEMOBUS data part
    // Length from MFC to end of data
    sbuf[12] = (unsigned char)(MDATAi & 0x00FF); // MEMOBUS data length (L)
    sbuf[13] = (unsigned char)((MDATAi & 0xFF00) >> 8); // MEMOBUS data length (H)
    // MFC is always 20 hex
    sbuf[14] = 0x20;
    // SFC
    sbuf[15] = SFC;
    // Set the CPU number
    sbuf[16] = (unsigned char)(CPUNum << 4); // The local CPU number is always 0 hex
    sbuf[17] = 0x00; // The spare is always 0 hex
    // Set the reference number
    sbuf[18] = (unsigned char)(Adr & 0x00FF); // Adr(L)
    sbuf[19] = (unsigned char)((Adr & 0xFF00) >> 8); // Adr(H)
    // Set the number of registers
    sbuf[20] = (unsigned char)(DataNum & 0x00FF); // DataNum(L)
    sbuf[21] = (unsigned char)((DataNum & 0xFF00) >> 8); // DataNum(H)
}

/
∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗/
/∗ Check response data ∗/
/∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗∗/
int chk_rsp_data(int rlen, char∗ sbuf, char∗ rbuf)
{
    short rcvDATAi; // Total number of data items to receive
    int rc = 0;
    // Check the total data length
    switch ( sbuf[15] )
    {
        case 0x09:
            rcvDATAi = 20 + ( (sbuf[21] << 8) | sbuf[20] ) ∗ 2;
            if ( rlen != rcvDATAi )
            {
                rc = -3;
                return( rc );
            }
            break;
        default:
            break;
    }
    // Check the packet type
    if ( rbuf[0] != 0x19 ) // Not a MEMOBUS response
    {
        rc = -4;
        return( rc );
    }
    // Check the serial number
    if ( sbuf[1] != rbuf[1] ) // Do not match the serial number of the command
    {
        rc = -5;
        return( rc );
    }
    //Check the total data length of the message
    if ( (rbuf[6] != (rcvDATAi & 0x00FF)) &&
    (rbuf[7] != (rcvDATAi & 0xFF00) >> 8) ) // ? bytes = 218 header (12 bytes)
    { // + MEMOBUS data (always 8 bytes + variable DataNum ∗ 2 bytes)
        rc = -6;
        return( rc );
    }
    // Check the total MEMOBUS data length
    // Check the MFC
    if ( rbuf[14] != 0x20 )// MFC is always 20 hex
    {
        rc = -7;
        return( rc );
    }
    // Check the SFC
    if ( rbuf[15] != sbuf[15] )
    {
        rc = -8;
        return( rc );
    }
    // Check the number of registers
    switch ( sbuf[15] )
    {
        case 0x09:
            if (( rbuf[18] != sbuf[20] ) || (rbuf[19] != sbuf[21] ))
            {
                rc = -9;
                return( rc );
            }
            break;
        default:
            break;
    }
    return( rc );
}