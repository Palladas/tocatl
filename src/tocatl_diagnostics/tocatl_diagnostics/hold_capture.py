"""Raw hip/knee CAN capture with optional, allowlisted register reads only.

No driver ownership, motor enable, stop, control, zero or parameter writes.
Local CyberGearDriver/parameters.py supplies register layouts. Tick units are
unverified; counter discontinuities are evidence, not a reset diagnosis alone.
"""
import argparse
from bisect import bisect_left, bisect_right
from collections import defaultdict
import json
import math
from pathlib import Path
import struct
import time

REGISTERS = {
    0x301e: ('iq_filtered_a', '<f'),
    0x3020: ('iq_a', '<f'),
    0x302d: ('torque_feedback_nm', '<f'),
    0x3007: ('bus_voltage_mv', '<H'),
    0x301c: ('tick_raw', '<I'),
    0x3022: ('fault_status', '<I'),
    0x3023: ('warning_status', '<I'),
    0x3024: ('driver_fault', '<H'),
    0x302b: ('bus_voltage_float', '<f'),
    0x2007: ('torque_limit_setting', '<f'),
    0x200c: ('can_timeout_setting', '<I'),
    0x2019: ('current_limit_setting', '<f'),
}
MOTOR_IDS = (12, 13, 127)


def read_request(motor_id, address):
    if motor_id not in MOTOR_IDS or address not in REGISTERS:
        raise ValueError('Probe only permits allowlisted reads of motors 12/13/127')
    return (9 << 24) | motor_id, struct.pack('<H', address)+bytes(6)


def decode(can_id, data):
    data=bytes(data)
    cmd=(can_id>>24)&31;destination=can_id&255;source=(can_id>>8)&255
    row=dict(type=cmd,destination=destination,source=source)
    if len(data)!=8:return row
    if cmd==2 and destination==0 and source in MOTOR_IDS:
        p,v,t,temp=struct.unpack('>4H',data)
        row.update(position_rad=p*25/65535-12.5,velocity_rad_s=v*60/65535-30,
                   torque_nm=t*24/65535-12,temperature_c=temp/10,
                   mode=(can_id>>22)&3,state_fault_bits=(can_id>>16)&63)
    elif cmd==9:
        addr=int.from_bytes(data[:2],'little');row['register']=hex(addr)
        if destination==0 and source in MOTOR_IDS and addr in REGISTERS:
            name,fmt=REGISTERS[addr];value=struct.unpack_from(fmt,data,4)[0]
            row.update(register_name=name,value=value if math.isfinite(value) else None)
    elif cmd==1 and destination in MOTOR_IDS:
        p,v,kp,kd=struct.unpack('>4H',data)
        row.update(command_position_rad=p*25/65535-12.5,command_velocity_rad_s=v*60/65535-30,
                   feedforward_nm=((can_id>>8)&65535)*24/65535-12,
                   kp=kp*500/65535,kd=kd*5/65535)
    return row


def analyze(path):
    rows=[]
    with Path(path).open() as f:
        for line in f:
            r=json.loads(line)
            if 'type' in r:rows.append(r)
    rows.sort(key=lambda r:r['time'])
    times=[r['time'] for r in rows]
    channels=defaultdict(list);volts=defaultdict(list);ticks=defaultdict(list)
    for r in rows:
        mid=r['source']
        if r['destination']!=0 or mid not in MOTOR_IDS:continue
        if r['type']==2:channels[(mid,'state')].append(r)
        if r.get('register_name')=='fault_status':channels[(mid,'fault_status')].append(r)
        if r.get('register_name')=='bus_voltage_mv' and r.get('value') is not None:
            volts[mid].append(r['value']/1000)
        if r.get('register_name')=='tick_raw' and r.get('value') is not None:ticks[mid].append(r)
    gaps=[]
    for (mid,kind),packets in channels.items():
        for a,b in zip(packets,packets[1:]):
            start,end=a['time'],b['time']
            if end-start<=.25:continue
            during=rows[bisect_right(times,start):bisect_left(times,end)]
            gaps.append(dict(motor=mid,kind=kind,start=start,end=end,gap_s=end-start,
                host_reads=sum(r['type'] in (2,9) and r['destination']==mid for r in during),
                other_motor_states=sum(r['type']==2 and r['destination']==0 and
                                       r['source'] in MOTOR_IDS and r['source']!=mid for r in during)))
    backwards=[]
    for mid,packets in ticks.items():
        for a,b in zip(packets,packets[1:]):
            if b['value']<a['value'] and not (a['value']>0xf0000000 and b['value']<0x0fffffff):
                backwards.append(dict(motor=mid,time=b['time'],before=a['value'],after=b['value']))
    return dict(frames=len(rows),voltage_v={mid:dict(min=min(v),max=max(v),samples=len(v)) for mid,v in volts.items()},
                tick_backwards=backwards,gaps_over_250ms=sorted(gaps,key=lambda r:r['start']),
                nonzero_fault_replies=[r for r in rows if r['destination']==0 and
                    r.get('register_name') in ('fault_status','warning_status','driver_fault') and r.get('value')])


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--channel',default='can1');p.add_argument('--output')
    p.add_argument('--hip-id',type=int,choices=(12,127),default=12)
    p.add_argument('--query',action='store_true',help='Read voltage/tick/warning registers; never command motion')
    p.add_argument('--electrical',action='store_true',help='Read selected hip motor q-axis current and torque feedback at 10 Hz; never command motion')
    p.add_argument('--duration',type=float,default=0,help='Seconds; zero captures until Ctrl+C')
    p.add_argument('--analyze');a=p.parse_args()
    if a.analyze:
        print(json.dumps(analyze(a.analyze),indent=2,allow_nan=False));return
    if not a.output:p.error('--output is required for capture')
    if not math.isfinite(a.duration) or a.duration<0:p.error('duration must be finite and nonnegative')
    import can
    path=Path(a.output);path.parent.mkdir(parents=True,exist_ok=True)
    # All probes are reads; source/host ID 0 matches the current controller.
    schedule=[(a.hip_id,0x3007,.1),(a.hip_id,0x301c,.2),(a.hip_id,0x3023,.5),(a.hip_id,0x3024,.5),
              (13,0x3007,.2),(13,0x301c,.2)]
    schedule = schedule if a.query else []
    if a.electrical:
        schedule += [(a.hip_id,addr,.1) for addr in (0x301e,0x3020,0x302d)]
    next_due={};started=time.monotonic()
    bus=can.Bus(interface='socketcan',channel=a.channel,receive_own_messages=True)
    try:
        with path.open('x',buffering=1) as f:
            f.write(json.dumps(dict(event='capture_started',time=time.time(),query=a.query,electrical=a.electrical,channel=a.channel))+'\n')
            print(f'Capture ready: {path}; read-only queries={bool(schedule)}',flush=True)
            while not a.duration or time.monotonic()-started<a.duration:
                now=time.monotonic()
                if schedule:
                    for mid,addr,period in schedule:
                        if now<next_due.get((mid,addr),0):continue
                        cid,data=read_request(mid,addr)
                        try:bus.send(can.Message(arbitration_id=cid,data=data,is_extended_id=True),timeout=.02)
                        except can.CanError as e:f.write(json.dumps(dict(event='probe_send_error',time=time.time(),error=str(e)))+'\n')
                        next_due[mid,addr]=now+period
                        break  # yield to receive between each read
                msg=bus.recv(.005)
                if msg is None:continue
                row=dict(time=msg.timestamp,received_unix_s=time.time(),received_monotonic_s=time.monotonic(),
                         can_id=hex(msg.arbitration_id),data=msg.data.hex(),is_rx=msg.is_rx,error_frame=msg.is_error_frame)
                if msg.is_extended_id and not msg.is_error_frame:
                    row.update(decode(msg.arbitration_id,msg.data))
                    if row['destination'] not in (a.hip_id,13) and row['source'] not in (a.hip_id,13):continue
                f.write(json.dumps(row,allow_nan=False)+'\n')
    except KeyboardInterrupt:pass
    finally:bus.shutdown()


if __name__=='__main__':main()
