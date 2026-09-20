
# delete artifacts from previous runs, if any
file(REMOVE mySolution.sol)
file(REMOVE mySolution.sol.PG.csv)

# solve the given instance
execute_process(
        COMMAND ./hgs ../Instances/CVRP/${INSTANCE}.vrp mySolution.sol -seed 1 -round ${ROUND}
        RESULTS_VARIABLE result
)

# a failed run must be reported as such, rather than as a missing solution file below
if(NOT result STREQUAL "0")
    message(FATAL_ERROR "Test error for ${INSTANCE}. hgs exited with: ${result}")
endif()

# read the result and compare with the given cost value
file(STRINGS mySolution.sol solution)
string(FIND "${solution}" "${COST}" cost_position)

# if not match, throw an error
if(${cost_position} EQUAL -1)
    message(SEND_ERROR "Test error for ${INSTANCE}. Cost != ${COST}")
endif()
