#include <iostream>
#include <vector>
#include <cstdint>
#include <cassert>
#include <stdexcept>

#include <string>
#include <fstream>
#include <streambuf>
#include <bitset>

#include <map>
#include <tr1/functional>
#include <tr1/unordered_map>

#include <boost/any.hpp>
#include <boost/lexical_cast.hpp>

using namespace std;

template <typename T>
T swap_endian(const T& t) {
    uint8_t byte_count = sizeof(T);
    T byte_mask = 0xFF;
    T result = 0;

    for (uint8_t i = 0; i < byte_count; i++) {
        T mask = (byte_mask << (i*8));
        T val = ((t & mask) >> i*8);
        uint8_t offset = (byte_count - i - 1)*8;
        T dest_mask = byte_mask << offset;
        result |= ((( val ) << offset) & dest_mask);
    }

    return result;
}

template <class T>
inline void hash_combine(std::size_t & seed, const T & v) {
    std::hash<T> hasher;
    seed ^= hasher(v) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
}

namespace std {
    namespace tr1 {
        template<typename S, typename T>
        struct hash<std::pair<S, T> > {
            inline size_t operator()(const pair<S, T> & v) const {
                  size_t seed = 0;
                  ::hash_combine(seed, v.first);
                  ::hash_combine(seed, v.second);
                  return seed;
            }
        };
    }
}

class RAM {
public:
    RAM(uint32_t size) {
        bytes_.resize(size);
    }

    template<typename T>
    T read(uint32_t address) {
        assert(address >= 0 && address < bytes_.size());
        if(!check_read_address(address)) {
            return 0;
        }
        return swap_endian(*((T*)&bytes_[address]));
    }

    template<typename T>
    void write(uint32_t address, T data) {
        assert(address >= 0 && address < bytes_.size());
        if(!check_write_address(address)) {
            return;
        }
        //Cast to pointer of type T then dereference to assign the value
        *((T*)&bytes_[address]) = swap_endian(data);
    }

    bool load_from_disk(const std::string& filename) {
        std::ifstream t(filename.c_str(), std::ios::binary);

        if(!t) {
            return false;
        }

        t.seekg(0, std::ios::end);
        bytes_.reserve(t.tellg());
        t.seekg(0, std::ios::beg);

        bytes_.assign((std::istreambuf_iterator<char>(t)),
                    std::istreambuf_iterator<char>());

        return true;
    }
private:
    virtual bool check_write_address(uint32_t address) { return true; }
    virtual bool check_read_address(uint32_t address) { return true; }
    std::vector<uint8_t> bytes_;
};

class ROM : public RAM {
public:
    ROM(uint32_t size):
        RAM(size) {}

private:
    bool check_write_address(uint32_t address) { return false; }
};

template<typename T>
class Register {
public:
    Register():
        value_(0) {}

    T read() { return value_; }
    void write(T value) {
        value_ = value;

        if(signal_changed) {
            signal_changed(value_);
        }
    }

    std::tr1::function<void (T)> signal_changed;
private:
    T value_;
};

class MemoryMap {
public:
    void add_range(uint32_t low, uint32_t high, RAM* block, uint32_t offset=0) {
        ranges_[std::make_pair(low, high)] = std::make_pair(block, offset);
    }

    template<typename T>
    void map_register(uint32_t address, T* reg) {
        registers_[address] = boost::any(reg);
    }

    template<typename T>
    void write(uint32_t address, T data) {
        for(RangeMap::const_iterator it = ranges_.begin(); it != ranges_.end(); ++it) {
            HighLow& range = (*it).first;
            if(address >= range.first && address < range.second) {
                RAMOffset& ram_offset = (*it).second;
                return ram_offset.first->write((address - range.first) + ram_offset.second, data);
            }
        }

        RegisterLookup::const_iterator it = registers_.find(address);
        if(it != registers_.end()) {
            boost::any_cast<Register<T>*>((*it).second)->write(data);
        }
    }

    template<typename T>
    T read(uint32_t address) {
        for(RangeMap::const_iterator it = ranges_.begin(); it != ranges_.end(); ++it) {
            const HighLow& range = (*it).first;
            if(address >= range.first && address < range.second) {
                const RAMOffset& ram_offset = (*it).second;
                return ram_offset.first->read<T>((address - range.first) + ram_offset.second);
            }
        }

        RegisterLookup::const_iterator it = registers_.find(address);
        if(it != registers_.end()) {
            return boost::any_cast<Register<T>*>((*it).second)->read();
        }

        throw std::out_of_range("Tried to access an invalid memory address");
    }

private:
    typedef std::pair<uint32_t, uint32_t> HighLow;
    typedef std::pair<RAM*, uint32_t> RAMOffset;
    typedef std::tr1::unordered_map<HighLow, RAMOffset> RangeMap;
    RangeMap ranges_;

    typedef std::tr1::unordered_map<uint32_t, boost::any> RegisterLookup;
    RegisterLookup registers_;
};

class CPU {
public:
    CPU() {

    }

    MemoryMap& memory() { return memory_map_; }

    virtual void tick() = 0;

    Register<uint32_t> PC;

private:
    MemoryMap memory_map_;


};

void noop(uint16_t opcode) {
    std::cout << std::bitset<16>(opcode).to_string() << std::endl;
}

typedef std::tr1::function<void (uint16_t)> OpcodeCallback;

uint8_t source_mode(uint16_t opcode) {
    uint16_t mode_mask = std::bitset<16>("0000000000111000").to_ulong();
    uint16_t result = (opcode & mode_mask) >> 3;
    return result;
}

uint8_t source_register(uint16_t opcode) {
    uint16_t mode_mask = std::bitset<16>("0000000000000111").to_ulong();
    uint16_t result = (opcode & mode_mask);
    return result;
}

uint8_t dest_register(uint16_t opcode) {
    uint16_t mode_mask = std::bitset<16>("0000111000000000").to_ulong();
    uint16_t result = (opcode & mode_mask) >> 9;
    return result;
}

class M68K : public CPU {
private:
    Register<uint32_t>* data_registers_[8];
    Register<uint32_t>* address_registers_[8];

    typedef std::map<uint16_t, OpcodeCallback > OpcodeTable;

    OpcodeTable opcode_lookup_;

    template<typename T>
    T read_address(uint16_t opcode) {
        uint8_t mode = source_mode(opcode);
        uint8_t source_reg = source_register(opcode);

        switch(mode) {
            case 2: return address_reg(source_reg).read();
            case 3: return address_reg(source_reg).read();
            case 4: return address_reg(source_reg).read();
            case 5: {
                T addr = address_reg(source_reg).read();
                addr += memory().read<uint16_t>(PC.read());
                PC.write(PC.read() + sizeof(uint16_t));
                return addr;
            }
            case 6: assert(0 && "Index not implemented");
            case 7: {
                switch(source_reg) {
                    case 0: {
                        T addr = memory().read<uint16_t>(PC.read());
                        PC.write(PC.read() + sizeof(uint16_t));
                        return addr;
                    }
                    case 1: {
                        T addr = memory().read<uint32_t>(PC.read());
                        PC.write(PC.read() + sizeof(uint32_t));
                        return addr;
                    }
                    case 2: {
                        T addr = PC.read();
                        addr += memory().read<uint16_t>(PC.read());
                        PC.write(PC.read() + sizeof(uint16_t));
                        return addr;
                    }
                    case 3: assert(0 && "Not implemented");
                    default:
                        assert(0 && "Invalid addressing mode");
                }
            }
            default:
                assert(0 && "Invalid addressing mode");
        }
    }

    template<typename T>
    T read_source_value(uint16_t opcode) {
        T source_value = 0;
        uint8_t mode = source_mode(opcode);
        uint8_t source_reg = source_register(opcode);
        switch(mode) {
            case 0: {
                //Read from data register
                source_value = data_reg(source_reg).read();
            } break;
            case 1: {
                //Read from address register
                source_value = address_reg(source_reg).read();
            } break;
            case 2:
            case 5:
            case 6: {
                //Read address from register and get value from memory
                source_value = memory().read<T>(read_address<T>(opcode));
            } break;
            case 3: {
                //Read address from register, get value from memory, then post-increment
                //address register
                T addr = read_address<T>(opcode);
                source_value = memory().read<T>(addr);
                address_reg(source_reg).write(addr + sizeof(T));
            } break;
            case 4: {
                //Read from address with pre-decrement
                uint32_t address = read_address<T>(opcode);
                address -= sizeof(T);
                address_reg(source_reg).write(address);
                source_value = memory().read<T>(address);
            } break;
            case 7: {
                switch(source_reg) {
                    case 4: {
                        //Immediate
                        source_value = (T) memory().read<uint16_t>(PC.read());
                        PC.write(PC.read() + sizeof(uint16_t)); //Move the program counter to the next instruction
                    } break;
                    default:
                        source_value = memory().read<T>(read_address<T>(opcode));
                }
            }
        }
        return source_value;
    }

    void _generate_opcode_variations(const std::string& first_ten_bits, OpcodeCallback cb) {
        //Generates all combinations of mode/source opcodes

        for(uint8_t M = 0; M < 7; ++M) {
            std::bitset<3> mval(M);
            for(uint8_t D = 0; D < 8; ++D) {
                std::bitset<3> dreg(D);

                uint16_t opcode = std::bitset<16>(first_ten_bits + mval.to_string() + dreg.to_string()).to_ulong();
                opcode_lookup_[opcode] = cb;
            }
        }

        for(uint8_t X = 0; X < 5; ++X) {
            std::bitset<3> xval(X);
            uint16_t opcode = std::bitset<16>(first_ten_bits + "111" + xval.to_string()).to_ulong();
            opcode_lookup_[opcode] = cb;
        }
    }

    void _generate_lea_opcodes() {
        OpcodeCallback lea_handler = std::bind(&M68K::lea, this, std::tr1::placeholders::_1);

        //Build all combinations of the LEA instruction
        for(uint8_t A = 0; A < 8; ++A) {
            std::bitset<3> areg(A);
            _generate_opcode_variations("0100" + areg.to_string() + "111", lea_handler);
        }
    }

    void _generate_movem_opcodes() {
        OpcodeCallback movem_word_handler = std::bind(&M68K::movem<uint16_t>, this, std::tr1::placeholders::_1);
        OpcodeCallback movem_long_handler = std::bind(&M68K::movem<uint32_t>, this, std::tr1::placeholders::_1);

        for(uint8_t D = 0; D < 2; ++D) {
            std::string d = (D) ? "1" : "0";
            std::string first_bits = string("01001") + d + string("001") + string("0");
            _generate_opcode_variations(first_bits, movem_word_handler);

            first_bits = string("01001") + d + string("001") + string("1");
            _generate_opcode_variations(first_bits, movem_long_handler);
        }
    }

public:
    Register<uint32_t> D0;
    Register<uint32_t> D1;
    Register<uint32_t> D2;
    Register<uint32_t> D3;
    Register<uint32_t> D4;
    Register<uint32_t> D5;
    Register<uint32_t> D6;
    Register<uint32_t> D7;

    Register<uint32_t> A0;
    Register<uint32_t> A1;
    Register<uint32_t> A2;
    Register<uint32_t> A3;
    Register<uint32_t> A4;
    Register<uint32_t> A5;
    Register<uint32_t> A6;
    Register<uint32_t> A7;

    Register<uint32_t>& SP; //Stack Pointer OR A7
    Register<uint16_t> SR; //Status register

    M68K():
        data_registers_{ &D0, &D1, &D2, &D3, &D4, &D5, &D6, &D7 },
        address_registers_{ &A0, &A1, &A2, &A3, &A4, &A5, &A6, &A7 },
        SP(A7) {

        build_opcode_table();
    }

    Register<uint32_t>& data_reg(uint8_t which) { return *data_registers_[which]; }
    Register<uint32_t>& address_reg(uint8_t which) { return *address_registers_[which]; }

    void reset() {
        SP.write(memory().read<uint32_t>(0));
        PC.write(memory().read<uint32_t>(4));
    }

    void lea(uint16_t opcode) {
        std::cout << "LEA: " << std::bitset<16>(opcode).to_string() << std::endl;

        uint8_t dest_reg = dest_register(opcode);
        uint32_t source_value = read_address<uint32_t>(opcode);
        address_reg(dest_reg).write(source_value);
    }

    template<typename T>
    void movem(uint16_t opcode) {
        std::cout << "MOVEM: " << std::bitset<16>(opcode).to_string() << std::endl;


        bool to_register = std::bitset<16>(opcode)[4];

        uint8_t mode = (opcode >> 3) & 7;
        uint8_t source_reg = opcode & 7;

        //Read the register mask from the program counter
        std::bitset<16> register_mask(memory().read<uint16_t>(PC.read()));
        PC.write(PC.read() + sizeof(uint16_t));

        for(uint8_t i = 0; i < 16; ++i) {
            if(register_mask[i]) {
                T value = read_source_value<T>(opcode);

                Register<uint32_t>& reg = (i > 7) ? address_reg(i - 8) : data_reg(i);

                if(to_register) {
                    reg.write(value);
                } else {

                }
            }
        }

    }

    void build_opcode_table() {
        const uint16_t RESET = std::bitset<16>("0100111001110000").to_ulong();
        const uint16_t NOP = std::bitset<16>("0100111001110001").to_ulong();
        const uint16_t STOP = std::bitset<16>("0100111001110010").to_ulong();
        const uint16_t RTE = std::bitset<16>("0100111001110011").to_ulong();
        const uint16_t RTS = std::bitset<16>("0100111001110001").to_ulong();
        const uint16_t TRAPV = std::bitset<16>("0100111001110110").to_ulong();
        const uint16_t RTR = std::bitset<16>("0100111001110111").to_ulong();

        opcode_lookup_ = std::map<uint16_t, OpcodeCallback >({
            { RESET, OpcodeCallback(noop) },
            { NOP, OpcodeCallback(noop) },
            { STOP, OpcodeCallback(noop) },
            { RTE, OpcodeCallback(noop) },
            { RTS, OpcodeCallback(noop) },
            { TRAPV, OpcodeCallback(noop) },
            { RTR, OpcodeCallback(noop) },
        });

        _generate_lea_opcodes();
        _generate_movem_opcodes();

    }

    void tick() {
        uint32_t addr = PC.read();
        uint16_t opcode = memory().read<uint16_t>(addr);
        PC.write(addr + sizeof(opcode)); //Immediately increment incase we need to read instruction data

        //Execute the opcode
        if(opcode_lookup_.find(opcode) != opcode_lookup_.end()) {
            opcode_lookup_[opcode](opcode);
        } else {
            std::cout << "UNHANDLED: " << std::bitset<16>(opcode).to_string() << std::endl;
        }

        //Handle interrupts
    }
};

class Z80 : public CPU {
public:
    Z80() {}

    void tick() {}

    Register<uint8_t> AF;
    Register<uint16_t> BC;
    Register<uint16_t> DE;
    Register<uint16_t> HL;
    Register<uint16_t> SP;

    Register<uint16_t> IX;
    Register<uint16_t> IY;
    Register<uint8_t> I;
    Register<uint8_t> R;
};


int main() {
    M68K cpu;
    Z80 cpu2;

    ROM tmss = ROM(2 * 1024); //BIOS
    ROM cartridge = ROM(4096 * 1024); //4M
    RAM m68k_ram = RAM(64 * 1024); //64k
    RAM z80_ram = RAM(8 * 1024); //8k
    RAM vram = RAM(64 * 1024); //64k

    //FIXME: add mappings for mirrored data
    cpu.memory().add_range(0x000000, 0x3FFFFF, &tmss);
    cpu.memory().add_range(0xFF0000, 0xFFFFFF, &m68k_ram);

    //Map the shared z80 ram
    cpu.memory().add_range(0xA00000, 0xA0FFFF, &z80_ram);
    cpu2.memory().add_range(0x000000, 0x00FFFF, &z80_ram);

    tmss.load_from_disk("../tmss.md");
    cartridge.load_from_disk("./sonic1.bin");

    cpu.reset();

    while(true) {
        cpu.tick();
    }

    return 0;
}

