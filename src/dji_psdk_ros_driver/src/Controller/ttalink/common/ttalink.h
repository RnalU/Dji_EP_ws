/** @file
 *  @brief TTAlink comm protocol built from common.xml
 *  @see http://ttalink.org
 */
#pragma once
#ifndef TTALINK_H
#define TTALINK_H

#define TTALINK_PRIMARY_XML_IDX 0

#ifndef TTALINK_STX
#define TTALINK_STX 85
#endif

#ifndef TTALINK_STX2
#define TTALINK_STX2 170
#endif

#ifndef TTALINK_STX2_NOCRC
#define TTALINK_STX2_NOCRC 171
#endif

#ifndef TTALINK_ENDIAN
#define TTALINK_ENDIAN TTALINK_LITTLE_ENDIAN
#endif

#ifndef TTALINK_ALIGNED_FIELDS
#define TTALINK_ALIGNED_FIELDS 0
#endif

#ifndef TTALINK_CRC_EXTRA
#define TTALINK_CRC_EXTRA 1
#endif

#ifndef TTALINK_COMMAND_24BIT
#define TTALINK_COMMAND_24BIT 0
#endif

#include "version.h"
#include "common.h"

#endif // TTALINK_H
