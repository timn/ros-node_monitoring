uint8 STARTING   = 0 # node is currently started and not yet functional
uint8 RUNNING    = 1 # node is fully operational
uint8 RECOVERING = 2 # node is recovering and currently not functional
uint8 ERROR      = 3 # node had an error and will try to recover
uint8 FATAL      = 4 # node had a fatal error and is disfunctional, restart req
uint8 STOPPING   = 5 # node is no longer functional and will soon exit

string nodename
time   time
uint8  state
string message

