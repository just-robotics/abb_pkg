MODULE ModuleGRIP
    
    VAR socketdev udp_socket;
    VAR string recv_str;
    VAR string sender_ip;
    VAR num sender_port;
    
    VAR bool signal_value := FALSE;

    PROC main()
        
        SocketCreate udp_socket \UDP;

        SocketBind udp_socket, "192.168.125.1", 6516;

        TPWrite "UDP listening on 6516";

        WHILE TRUE DO
            
            SocketReceiveFrom udp_socket \Str:=recv_str, sender_ip, sender_port;

            IF StrLen(recv_str) > 0 THEN
                
                IF StrPart(recv_str,1,1) = "1" THEN
                    signal_value := TRUE;
                ELSEIF StrPart(recv_str,1,1) = "0" THEN
                    signal_value := FALSE;
                ENDIF
                
            ENDIF
            
            IF signal_value THEN
                PulseDO\High, do_GripClose;
            ELSE
                PulseDO\High, do_GripOpen;
            ENDIF

            TPWrite "Getting " + recv_str;

        ENDWHILE
        
    ERROR
        IF ERRNO = ERR_SOCK_TIMEOUT THEN
            TPWrite "Communication timedout";
            TRYNEXT;
        ENDIF

    ENDPROC
    
ENDMODULE

