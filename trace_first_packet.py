"""
DIAGNOSTIC SCRIPT: Trace First TORCS Packet

This script adds comprehensive logging to trace the exact first control packet
sent to TORCS after connection is established.

Run this BEFORE the actual experiment to capture diagnostic data.
"""

import sys
import os

# Add logging patches to snakeoil3_gym.py
SNAKEOIL_PATCH = '''
# ============ DIAGNOSTIC LOGGING PATCH ============
import sys
_packet_count = 0

def _log_packet(location, data):
    global _packet_count
    _packet_count += 1
    print(f"\\n{'='*60}", file=sys.stderr)
    print(f"PACKET #{_packet_count} at {location}", file=sys.stderr)
    print(f"{'='*60}", file=sys.stderr)
    if isinstance(data, dict):
        for k, v in sorted(data.items()):
            print(f"  {k:12s} = {v!r}", file=sys.stderr)
    else:
        print(f"  {data!r}", file=sys.stderr)
    print(f"{'='*60}\\n", file=sys.stderr)
    sys.stderr.flush()
# ============ END DIAGNOSTIC PATCH ============
'''

# Patch locations in snakeoil3_gym.py
RESPOND_TO_SERVER_PATCH = '''
    def respond_to_server(self):
        if not self.so: return
        try:
            _log_packet("BEFORE clip_to_limits", self.R.d.copy())  # DIAGNOSTIC
            message = repr(self.R)
            _log_packet("AFTER clip_to_limits (final packet)", self.R.d.copy())  # DIAGNOSTIC
            if self.trace_packets:
                print("TORCS_PACKET:", message)
            print(f"\\nSENDING TO TORCS: {message}\\n", file=sys.stderr)  # DIAGNOSTIC
            sys.stderr.flush()
            self.so.sendto(message.encode(), (self.host, self.port))
            print(f"PACKET SENT SUCCESSFULLY\\n", file=sys.stderr)  # DIAGNOSTIC
            sys.stderr.flush()
        except socket.error as emsg:
            print(f"ERROR DURING SEND: {emsg}", file=sys.stderr)  # DIAGNOSTIC
            sys.stderr.flush()
            raise ConnectionError("Error sending to server: %s" % emsg) from emsg
        if self.debug: print(self.R.fancyout())
'''

# Patch for gym_torcs.py step function
GYM_STEP_PATCH = '''
        # DIAGNOSTIC: Log action before conversion
        print(f"\\nACTION FROM AGENT: {u}", file=sys.stderr)
        sys.stderr.flush()
        
        this_action = self.agent_to_torcs(u)
        
        # DIAGNOSTIC: Log converted action
        print(f"CONVERTED ACTION: {this_action}", file=sys.stderr)
        sys.stderr.flush()

        # Apply Action
        action_torcs = client.R.d
        
        # DIAGNOSTIC: Log R.d BEFORE modification
        print(f"\\nR.d BEFORE modification:", file=sys.stderr)
        for k, v in sorted(client.R.d.items()):
            print(f"  {k:12s} = {v!r}", file=sys.stderr)
        sys.stderr.flush()
'''

# Patch for autom_agent.py
AGENT_PATCH = '''
        # DIAGNOSTIC: Log observation fields
        print(f"\\n{'='*60}", file=sys.stderr)
        print(f"OBSERVATION RECEIVED:", file=sys.stderr)
        print(f"  Fields: {ob._fields if hasattr(ob, '_fields') else 'N/A'}", file=sys.stderr)
        print(f"  Has gear: {hasattr(ob, 'gear')}", file=sys.stderr)
        if hasattr(ob, 'gear'):
            print(f"  ob.gear = {ob.gear}", file=sys.stderr)
        else:
            print(f"  ob.gear = MISSING (will use fallback)", file=sys.stderr)
        print(f"  previous_command.gear = {self.previous_command.gear}", file=sys.stderr)
        sys.stderr.flush()
        
        sensor_data = {
            "track": ob.track,
            "angle": ob.angle,
            "trackPos": ob.trackPos,
            "speedX": ob.speedX,
            "speedY": ob.speedY,
            "speedZ": ob.speedZ,
            "rpm": ob.rpm,
            "wheelSpinVel": ob.wheelSpinVel,
            "gear": getattr(ob, "gear", self.previous_command.gear),
            "curLapTime": getattr(ob, "curLapTime", 0.0),
            "distFromStart": getattr(ob, "distFromStart", 0.0),
        }
        
        # DIAGNOSTIC: Log sensor_data
        print(f"\\nSENSOR_DATA TO CONTROLLER:", file=sys.stderr)
        print(f"  gear (used) = {sensor_data['gear']}", file=sys.stderr)
        print(f"  speedX = {sensor_data['speedX']}", file=sys.stderr)
        print(f"  rpm = {sensor_data['rpm']}", file=sys.stderr)
        sys.stderr.flush()

        command = self.controller.update(
            sensor_data=sensor_data,
            previous_command=self.previous_command,
        )
        
        # DIAGNOSTIC: Log command
        print(f"\\nCOMMAND FROM CONTROLLER:", file=sys.stderr)
        print(f"  steering = {command.steering}", file=sys.stderr)
        print(f"  acceleration = {command.acceleration}", file=sys.stderr)
        print(f"  brake = {command.brake}", file=sys.stderr)
        print(f"  gear = {command.gear}", file=sys.stderr)
        print(f"{'='*60}\\n", file=sys.stderr)
        sys.stderr.flush()

        self.previous_command = command

        return [
            command.steering,
            command.acceleration,
            command.brake,
            command.gear,
        ]
'''

print("""
DIAGNOSTIC SCRIPT FOR TORCS CRASH INVESTIGATION
================================================

This script provides the exact patches needed to trace the first packet.

MANUAL PATCHING INSTRUCTIONS:
==============================

1. In snakeoil3_gym.py:
   - Add the diagnostic logging patch at the top (after imports)
   - Replace respond_to_server() method with patched version

2. In gym_torcs.py:
   - Add diagnostic logging in step() method before agent_to_torcs()

3. In autom_agent.py:
   - Add diagnostic logging in act() method

4. Run: python3 example_experiment.py 2> first_packet_trace.log

5. Examine first_packet_trace.log for exact packet values

ALTERNATIVELY: Use the logging statements below to manually add to code.

""")

print("\n" + "="*60)
print("SNAKEOIL3_GYM.PY PATCH (add after imports):")
print("="*60)
print(SNAKEOIL_PATCH)

print("\n" + "="*60)
print("SNAKEOIL3_GYM.PY respond_to_server() REPLACEMENT:")
print("="*60)
print(RESPOND_TO_SERVER_PATCH)

print("\n" + "="*60)
print("GYM_TORCS.PY step() PATCH (add at start of step method):")
print("="*60)
print(GYM_STEP_PATCH)

print("\n" + "="*60)
print("AUTOM_AGENT.PY act() PATCH (replace sensor_data creation):")
print("="*60)
print(AGENT_PATCH)

print("\n" + "="*60)
print("EXPECTED OUTPUT FORMAT:")
print("="*60)
print("""
The trace will show:

1. OBSERVATION RECEIVED
   - Fields available in observation
   - Whether 'gear' field exists
   - Fallback values used

2. SENSOR_DATA TO CONTROLLER
   - Exact gear value sent to controller
   - Speed, RPM values

3. COMMAND FROM CONTROLLER
   - steering, acceleration, brake, gear

4. PACKET #1 at BEFORE clip_to_limits
   - Raw values before validation

5. PACKET #1 at AFTER clip_to_limits
   - Final values after validation
   - Shows: steer, accel, brake, clutch, gear, focus, meta

6. SENDING TO TORCS: (accel X)(brake X)(gear X)(steer X)(clutch X)(focus ...)(meta X)

7. PACKET SENT SUCCESSFULLY
   OR
   ERROR DURING SEND: <error message>

This will prove:
- Exact values in first packet
- Whether crash is before/during/after send
- Whether values are finite and valid
""")
