


const uint32_t TTN_NET_ID = 0x000013;
const uint32_t DEV_ADRESS = 0x260B4A6F;
const uint8_t NET_KEY[] = {0x1C, 0x76, 0xD5, 0x82, 0x65, 0xB5, 0x7C, 0xA0, 
                           0xBE, 0x78, 0x69, 0xC9, 0x81, 0x99, 0x1D, 0xC5};     // 1C76D58265B57CA0BE7869C981991DC5
const uint8_t APP_KEY[] = {0x9B, 0x25, 0x5A, 0xD5, 0x47, 0x5F, 0x8D, 0x1D, 
                           0xBF, 0x16, 0x8B, 0x41, 0xA9, 0x3D, 0x28, 0xBE };    // 9B255AD5475F8D1DBF168B41A93D28BE
                           

/* GPS coordinates of mapped gateway for calculating the distance */
// Eastend Tower
// const double HOME_LAT = 52.005080;
// const double HOME_LNG =  8.560120;

// Uphofweg
const double HOME_LAT = 51.964605;
const double HOME_LNG =  8.711868;
