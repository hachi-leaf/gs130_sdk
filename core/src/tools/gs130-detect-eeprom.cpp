/* gs130-detect-eeprom: scan I2C buses x addresses for a known EEPROM, printing
 * a result for every probe point (name + info on a hit). Internal bring-up tool. */
#include "base/i2c/i2c.hpp"
#include "devices/eeprom/eeprom.hpp"

#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <vector>

using namespace gs130;

static bool parse_int(const char *s, int *out)
{
    char *end = nullptr;
    long v = strtol(s, &end, 0);   // base 0: decimal, 0x-hex, 0-octal
    if(end == s || *end != '\0')return false;
    *out = (int)v;
    return true;
}

static void print_block(const char *indent, const char *text)
{
    if(!text)return;
    for(const char *p = text; *p; ){
        const char *nl = strchr(p, '\n');
        int len = nl ? (int)(nl - p) : (int)strlen(p);
        printf("%s%.*s\n", indent, len, p);
        if(!nl)break;
        p = nl + 1;
    }
}

int main(int argc, char **argv)
{
    std::vector<int> buses, addrs;
    for(int i = 1; i < argc; i++){
        if(!strcmp(argv[i], "-b") || !strcmp(argv[i], "-a")){
            std::vector<int> &dst = (argv[i][1] == 'b') ? buses : addrs;
            while(i + 1 < argc && argv[i+1][0] != '-'){
                int v;
                if(!parse_int(argv[++i], &v)){ fprintf(stderr, "bad number: %s\n", argv[i]); return 1; }
                dst.push_back(v);
            }
        }
    }
    if(buses.empty()){
        fprintf(stderr, "usage: gs130 detect eeprom -b <bus...> [-a <addr...>]   (addr default: 0x50)\n");
        return 1;
    }
    if(addrs.empty())addrs = {0x50};

    printf("EEPROM scan: buses");
    for(int b : buses)printf(" %d", b);
    printf(", addrs");
    for(int a : addrs)printf(" 0x%02x", a);
    printf("\n bus  addr    result\n ---  -----   ----------------\n");

    int n_found = 0, n_unknown = 0, n_empty = 0, n_buserr = 0;
    for(int b : buses){
        base::I2cDevice probe((uint8_t)b, (uint8_t)addrs.front());
        if(!probe){
            printf(" %3d  --      bus open failed\n", b);
            n_buserr++;
            continue;
        }
        for(int a : addrs){
            // constructor probes the auto-registered model table on this bus/addr
            eeprom::Eeprom ep((uint8_t)b, (uint8_t)a);
            if(ep){
                printf(" %3d  0x%02x   %s\n", b, a, ep.name());
                print_block("                ", ep.info());
                n_found++;
            } else {
                base::I2cDevice d((uint8_t)b, (uint8_t)a);
                uint8_t v;
                if(d && d.read(0x00, &v) == Status::Ok){
                    printf(" %3d  0x%02x   unknown device\n", b, a);
                    n_unknown++;
                } else {
                    printf(" %3d  0x%02x   no device\n", b, a);
                    n_empty++;
                }
            }
        }
    }
    printf("found %d, unknown %d, empty %d, bus error %d\n", n_found, n_unknown, n_empty, n_buserr);
    return 0;
}
