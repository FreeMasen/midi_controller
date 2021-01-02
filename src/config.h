
struct Config
{
    uint8_t first_midi_note;
    int cc;
};

Config new_config()
{
    Config cfg;
    cfg.first_midi_note = 36;
    cfg.cc = 1;
    return cfg;
}

typedef uint8_t (*ConfigFn)(Config *config);

uint8_t set_midi_24(Config *config)
{
    config->first_midi_note = 24;
    return 1;
}

uint8_t set_midi_36(Config *config)
{
    config->first_midi_note = 36;
    return 2;
}

ConfigFn fns[255] = {
    set_midi_24,
    set_midi_36
};

uint8_t update_config(Config *config, uint8_t btn)
{
    if (fns[btn])
    {
        return fns[btn](config);
    }
    return 0;
}
