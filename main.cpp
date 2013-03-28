#include <iostream>
#include <vector>
#include <cstdint>
#include <cassert>
#include <stdexcept>

#include <string>
#include <fstream>
#include <streambuf>

#include <tr1/functional>
#include <tr1/unordered_map>

#include <boost/any.hpp>

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

class M68K : public CPU {
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
        SP(A7) {}

    void reset() {
        SP.write(memory().read<uint32_t>(0));
        PC.write(memory().read<uint32_t>(4));
    }

    void tick() {
        //Is the opcode 32 bit?
        uint32_t addr = PC.read();
        uint16_t opcode = memory().read<uint16_t>(addr);

        switch(opcode) {

        }

        PC.write(addr + sizeof(opcode));

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

