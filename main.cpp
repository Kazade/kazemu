#include <iostream>
#include <vector>

using namespace std;

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
        return *((T*)&bytes_[address]);
    }

    template<typename T>
    void write(uint32_t address, T data) {
        assert(address >= 0 && address < bytes_.size());
        if(!check_write_address(address)) {
            return;
        }
        //Cast to pointer of type T then dereference to assign the value
        *((T*)&bytes_[address]) = data;
    }

    bool load_from_disk(const std::string& filename) {

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

class MemoryMap {
public:
    void add_range(uint32_t low, uint32_t high, RAM* block, uint32_t offset=0) {
        ranges_[std::make_pair(low, high)] = std::make_pair(block, offset);
    }

    template<typename T>
    void write(uint32_t address, T data) {
        for(RangeMap::const_iterator it = ranges_.begin(); it != ranges_.end(); ++it) {
            HighLow& range = (*it).first;
            if(address >= range.first && range < range.second) {
                RAMOffset& ram_offset = (*it).second;
                return ram_offset.first->write(address + ram_offset.second, data);
            }
        }
    }

    template<typename T>
    T read(uint32_t address) {
        for(RangeMap::const_iterator it = ranges_.begin(); it != ranges_.end(); ++it) {
            HighLow& range = (*it).first;
            if(address >= range.first && range < range.second) {
                RAMOffset& ram_offset = (*it).second;
                return ram_offset.first->read(address + ram_offset.second);
            }
        }

        return 0;
    }

private:
    typedef std::pair<uint32_t, uint32_t> HighLow;
    typedef std::pair<RAM*, uint32_t> RAMOffset;
    typedef std::unordered_map<HighLow, RAMOffset> RangeMap;
    RangeMap ranges_;
};

template<typename T>
class Register {
public:
    Register():
        value_(0) {}

    T read() { return value_; }
    void write(T value) { value_ = value; }

private:
    T value_;
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

    Register<uint32_t> SP; //Stack Pointer OR A7
    Register<uint16_t> SR; //Status register

    M68K() {}

    void tick() {
        //Is the opcode 32 bit?
        uint32_t opcode = memory().read(PC.read());

        switch(opcode) {

        }

        PC.write(PC.read() + 4); //Should this be 4?

        //Handle interrupts
    }
};

class Z80 : public CPU {
public:
    Z80();

    void tick();

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
    cpu.memory().add_range(0x000000, 0x3FFFFF, &cartridge);
    cpu.memory().add_range(0xFF0000, 0xFFFFFF, &m68k_ram);

    //Map the shared z80 ram
    cpu.memory().add_range(0xA00000, 0xA0FFFF, &z80_ram);
    cpu2.memory().add_range(0x000000, 0x00FFFF, &z80_ram);

    cartridge.load_from_disk("./sonic1.bin");

    //Build vector table...
    cpu.SP.write(cpu.memory().read<uint32_t>(0x000000)); //First value is the stack pointer
    cpu.PC.write(cpu.memory().read<uint32_t>(0x000004)); //4 bytes in is the address for the program counter

    while(true) {
        cpu.tick();
    }

    return 0;
}

