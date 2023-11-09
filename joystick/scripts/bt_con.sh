#!/usr/bin/expect -f

#5 secs to wait answers
set timeout 5
set echo "-noecho"

#get mac from file
if {[catch {set f [open "~/braintemp/joy_mac.txt"]} err]} {
    puts "Open mac file failed: $err"
    exit 1
}
set MAC [read -nonewline $f]
close $f

if {$MAC eq ""} {
    puts "\nCouldn't find mac"
    exit 1
} else {
    puts "\nI find mac: $MAC, attempting to pair..."
}

proc clean_buffer {id} {
    #cleaning buffer
    set expect_out(0,string) ""
    set expect_out(spawn_id) "$id"
    set expect_out(buffer) ""
}

spawn -noecho bluetoothctl
sleep 2
set bt_id $spawn_id
send -- "default-agent\n"
expect -exact "Default agent request successful"
clean_buffer $bt_id
send -- "paired-devices\n"
expect -re "Device $MAC" {
    puts "\nDevice $MAC already paired, removing ..."
    send -- "remove $MAC\n"
    expect -exact "Device has been removed"
    clean_buffer $bt_id
}
sleep 1
#25 secs to scan & find device
set timeout 25
send -- "scan on\n"
expect -exact "Discovery started"

expect {
    #if it finds the devices try to pairing
    -re "Device $MAC" {
        send -- "trust $MAC\n"
        send -- "pair $MAC\n"

        expect {
            -re "Pairing successful" {
                clean_buffer $bt_id
                send -- "connect $MAC\n"
                expect {
                    -re "Connection successful" {
                        sleep 1
                        clean_buffer $bt_id
                        send -- "info $MAC\n"
                        expect {
                            "Connected: yes" {}
                        }
                    }
                }
            }
        }
    }
    #if not show a user message
    timeout {
        puts "\nCouldn't find $MAC"
        sleep 1
        send -- "quit\n"
        expect eof
        exit 1
    }
}

sleep 4
send -- "quit\n"
expect eof
exit 0
