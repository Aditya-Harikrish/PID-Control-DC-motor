#ifndef CONFIG_H
#define CONFIG_H

// WiFi
    #define WIFI_SSID "hackiiit"
    #define WIFI_PASSWD "12344321"

// ThingSpeak
    #define USER_API_KEY "CMGTYVOSQGX3XCHI"

    //Archive Channel ID
        #define ARCHIVE_CHANNEL_ID    1825221
        #define ARCHIVE_WRITE_API_KEY "XU89SKWS3ZIIHP52"
        #define ARCHIVE_READ_API_KEY  "U29GEZH7BZ4ECF53"
    
    //Live Channel ID
        #define LIVE_CHANNEL_ID    1856865
        #define LIVE_WRITE_API_KEY "S6W8OO70L5TBDX50"
        #define LIVE_READ_API_KEY  "CTF2SKY12O8L422M"
// Onem2m
    #define USER_PASS "KtSpH8:F902As"
    #define Post_URL " https://esw-onem2m.iiit.ac.in/~/in-cse/in-name/Team-35/Node-1/Data"

// PID constants
    #define K_P 10.0
    #define K_D 0.025
    #define K_I 5.0
#endif
