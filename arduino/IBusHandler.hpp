#ifndef IBUS_HANDLER_H
#define IBUS_HANDLER_H

#include <IBusBM.h>

int readIBusChannel(IBusBM& ibus, byte channelInput, float minLimit, float maxLimit, float defaultValue);

#endif // IBUS_HANDLER_H
