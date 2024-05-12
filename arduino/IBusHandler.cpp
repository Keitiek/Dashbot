#include <Arduino.h>
#include <IBusBM.h>
#include <stdlib.h>

#include "IBusHandler.hpp"
#include "FlySkyChannelDefinitions.hpp"

int readIBusChannel(IBusBM& ibus, byte channelInput, float minLimit, float maxLimit, float defaultValue) {
    uint16_t readValue = ibus.readChannel(channelInput);
    if (readValue < 100) return defaultValue;
    return map(readValue, 1000, 2000, minLimit, maxLimit);
}
