MODULE ModuleEGM
    !***********************************************************
    ! Program data
    !***********************************************************
    ! Home position.   
    LOCAL CONST jointtarget home := [[0, 0, 0, 0, 90, -90], [9E9, 9E9, 9E9, 9E9, 9E9, 9E9]];
    !LOCAL CONST jointtarget home := [[-2, 32, 10, 0, 46, -92], [9E9, 9E9, 9E9, 9E9, 9E9, 9E9]];
    
    ! Identifier for the EGM correction.
    LOCAL VAR egmident egm_id;
    
    ! Limits for convergance.
    LOCAL VAR egm_minmax egm_condition := [-0.1, 0.1];
    
    LOCAL CONST egm_minmax egm_minmax_lin:=[-1.0,1.0];
    LOCAL CONST egm_minmax egm_minmax_rot:=[-0.1,0.1];
    LOCAL CONST pose posecor:=[[0,0,0], [1,0,0,0]];
    LOCAL CONST pose posesens:=[[0,0,0], [1,0,0,0]];
    

    PROC main()
        !Path_circle;
        MoveAbsJ home, v200, fine, tool0;
        WHILE TRUE DO
            ! Register an EGM id.
            EGMGetId egm_id;
            
            
            ! Setup the EGM communication.
            !EGMSetupUC ROB_1, egm_id, "default", "UCdevice" \Joint;
            
            ! Prepare for an EGM communication session.
            !EGMActJoint egm_id
            !            \J1:=egm_condition
            !            \J2:=egm_condition
            !            \J3:=egm_condition
            !            \J4:=egm_condition
            !            \J5:=egm_condition
            !            \J6:=egm_condition
            !            \MaxSpeedDeviation:=1000.0
            !            ;
                        
                        
            ! Start the EGM communication session.
            ! Note: The position correction gain is set to 0 to allow for pure velocity motions.
            !EGMRunJoint egm_id, EGM_STOP_HOLD, \J1 \J2 \J3 \J4 \J5 \J6;
            
            
            ! Setup the EGM communication.
            EGMSetupUC ROB_1, egm_id, "default", "UCdevice" \Pose;
            
            ! Prepare for an EGM communication session.
            EGMActPose egm_id
                \Tool:=tool0
                \WObj:=wobj0, posecor, EGM_FRAME_BASE, tool0.tframe, EGM_FRAME_BASE
                \x:=egm_minmax_lin
                \y:=egm_minmax_lin
                \z:=egm_minmax_lin
                \rx:=egm_minmax_rot
                \ry:=egm_minmax_rot
                \rz:=egm_minmax_rot
                \SampleRate:=4
                \MaxSpeedDeviation:=100
                ;  
                        
            ! Start the EGM communication session.
            ! Note: The position correction gain is set to 0 to allow for pure velocity motions.
            EGMRunPose egm_id, EGM_STOP_HOLD, \x \y \z \rx \ry \rz;
            
            ! Release the EGM id.
            EGMReset egm_id;
        ENDWHILE
        MoveAbsJ home, v200, fine, tool0;
        
    ERROR
        IF ERRNO = ERR_UDPUC_COMM THEN
            TPWrite "Communication timedout";
            TRYNEXT;
        ENDIF
    ENDPROC
ENDMODULE