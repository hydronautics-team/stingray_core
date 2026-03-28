#include "test_hydrolib_sp_interpreter_env.hpp"
#include "hydrolib_common.h"
#include "hydrolib_logger.hpp"
#include "hydrolib_serial_protocol_commands.hpp"

inline TestLogStream log_stream;
inline LogDistributor distributor("[%s] [%l] %m\n", log_stream);
inline Logger logger("Serializer", 0, distributor);

TestHydrolibSerialProtocolInterpreter::TestHydrolibSerialProtocolInterpreter()
    : interpreter(public_memory, transmitter, logger)
{
    distributor.SetAllFilters(0, LogLevel::DEBUG);
    for (int i = 0; i < PUBLIC_MEMORY_LENGTH; i++)
    {
        test_data[i] = i;
    }
}

// INSTANTIATE_TEST_CASE_P(
//     Test, TestHydrolibSerialProtocolSerializeParametrized,
//     ::testing::Combine(::testing::Range<uint16_t>(0, PUBLIC_MEMORY_LENGTH),
//                        ::testing::Range<uint16_t>(1,
//                                                   PUBLIC_MEMORY_LENGTH +
//                                                   1)));
