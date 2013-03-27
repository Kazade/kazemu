#include <iostream>
#include <vector>

using namespace std;

class RAM {
public:
    RAM(uint32_t size);

    uint8_t* read(uint32_t address);
    void write(uint32_t address, uint8_t* data);

private:
    virtual bool check_write_address(uint32_t address) { return true; }
    virtual bool check_read_address(uint32_t address) { return true; }
    std::vector<uint8_t> bytes;
};

class ROM : public RAM {
public:
    ROM(uint32_t size);

private:
    bool check_write_address(uint32_t address) { return false; }
};

class MemoryMap {
public:
    void add_range(uint32_t low, uint32_t high, RAM* block, uint32_t offset=0);

    void write(uint32_t address, uint8_t* data) {

    }

    uint8_t* read(uint32_t address) {
        for(RangeMap::const_iterator it = ranges_.begin(); it != ranges_.end(); ++it) {
            HighLow& range = (*it).first;
            if(address >= range.first && range < range.second) {
                RAMOffset& ram_offset = (*it).second;
                return ram_offset.first->read(address + ram_offset.second);
            }
        }
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
    Register<uint32_t> SR; //Status register

    M68K() {}

    void tick() {
        uint32_t opcode = memory().read(program_counter_++);
        switch(opcode) {

        }
    }
};

class Z80 : public CPU {
public:
    Z80();

    void tick();
};


int main() {
    M68K cpu;
    Z80 cpu2;

    //ROM bios = ROM(2 * 1024); //BIOS
    ROM cartridge = ROM(4096 * 1024); //4M
    RAM m68k_ram = RAM(64 * 1024); //64k
    RAM z80_ram = RAM(8 * 1024); //8k

    //FIXME: add mappings for mirrored data
    cpu.memory().add_range(0x000000, 0x3FFFFF, &cartridge);
    cpu.memory().add_range(0xFF0000, 0xFFFFFF, &m68k_ram);

    //Map the shared z80 ram
    cpu.memory().add_range(0xA00000, 0xA0FFFF, &z80_ram);
    cpu2.memory().add_range(0x000000, 0x00FFFF, &z80_ram);

    uint32_t initial_stack_pointer = 0x00FFE000;
    uint32_t start_of_rom = 0x00000200;

    cpu.PC.write(start_of_rom);
    cpu.SP.write(initial_stack_pointer); //Apparently the MD sets this as the default stack pointer

    cartridge.load_from_disk("./sonic1.bin");

    while(true) {
        cpu.tick();
    }

    return 0;
}

