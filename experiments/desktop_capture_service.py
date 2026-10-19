#!/usr/bin/env python3
"""Capture complete monitor frames through the desktop portal."""
import gi,json,os,signal,sys,time,threading
from pathlib import Path
from PIL import Image
gi.require_version('Gst','1.0')
gi.require_version('GstVideo','1.0')
from gi.repository import Gio,GLib,Gst,GstVideo
folder=Path(sys.argv[1]).resolve();folder.mkdir(parents=True,exist_ok=True)
request_file=folder/'request.txt';ready=folder/'ready.json';ready.unlink(missing_ok=True)
bus=Gio.bus_get_sync(Gio.BusType.SESSION,None);responses={};waiting=None

def on_response(connection,sender,path,interface,name,parameters,user):
    responses[path]=parameters.unpack()
    if waiting is not None:waiting.quit()
bus.signal_subscribe(None,'org.freedesktop.portal.Request','Response',None,None,Gio.DBusSignalFlags.NONE,on_response,None)
def call(method,signature,args):
    global waiting
    token='lab2_'+str(time.time_ns());opts=args[-1];opts['handle_token']=GLib.Variant('s',token)
    reply=bus.call_sync('org.freedesktop.portal.Desktop','/org/freedesktop/portal/desktop','org.freedesktop.portal.ScreenCast',method,GLib.Variant(signature,args),None,Gio.DBusCallFlags.NONE,10000,None)
    path=reply.unpack()[0];deadline=time.monotonic()+600
    while path not in responses:
        if time.monotonic()>deadline:raise TimeoutError(method+' permission timed out')
        waiting=GLib.MainLoop();timer=GLib.timeout_add_seconds(1,lambda:(waiting.quit(),False)[1]);waiting.run()
        if GLib.MainContext.default().find_source_by_id(timer):GLib.source_remove(timer)
    waiting=None;code,data=responses.pop(path)
    if code!=0:raise RuntimeError(f'{method} cancelled or denied: {code}')
    return data
session=call('CreateSession','(a{sv})',({'session_handle_token':GLib.Variant('s','lab2_session_'+str(time.time_ns()))},))['session_handle']
call('SelectSources','(oa{sv})',(session,{'types':GLib.Variant('u',1),'multiple':GLib.Variant('b',False),'cursor_mode':GLib.Variant('u',2)}))
print('Select the complete monitor in the system screen-sharing dialog.',flush=True)
stream=call('Start','(osa{sv})',(session,'',{}))['streams'][0]
reply,fds=bus.call_with_unix_fd_list_sync('org.freedesktop.portal.Desktop','/org/freedesktop/portal/desktop','org.freedesktop.portal.ScreenCast','OpenPipeWireRemote',GLib.Variant('(oa{sv})',(session,{})),None,Gio.DBusCallFlags.NONE,10000,None,None)
fd=fds.get(reply.unpack()[0]);node_id=stream[0]
Gst.init(None)
pipeline=Gst.parse_launch(f'pipewiresrc fd={fd} path={node_id} do-timestamp=true ! videoconvert ! video/x-raw,format=RGB ! appsink name=frames emit-signals=true max-buffers=1 drop=true sync=false')
loop=GLib.MainLoop();state={'pending':str(folder/'probe.png'),'frames':0};lock=threading.Lock()
def frame(sink):
    sample=sink.emit('pull-sample')
    if sample is None:return Gst.FlowReturn.ERROR
    with lock:path=state['pending'];state['pending']=None
    if path:
        info=GstVideo.VideoInfo.new_from_caps(sample.get_caps());buf=sample.get_buffer();ok,mapping=buf.map(Gst.MapFlags.READ)
        if ok:
            try:
                im=Image.frombytes('RGB',(info.width,info.height),bytes(mapping.data),'raw','RGB',info.stride[0],1)
                target=Path(path);target.parent.mkdir(parents=True,exist_ok=True);im.save(target)
                ready.write_text(json.dumps({'ready':True,'width':info.width,'height':info.height,'session':session,'last_capture':path}))
                print('Saved full desktop: '+path,flush=True)
            finally:buf.unmap(mapping)
    return Gst.FlowReturn.OK
pipeline.get_by_name('frames').connect('new-sample',frame)
def poll():
    if request_file.exists():
        path=request_file.read_text().strip();request_file.unlink(missing_ok=True)
        if path:
            with lock:state['pending']=path
    if (folder/'stop').exists():loop.quit();return False
    return True
def gst_message(bus,message):
    if message.type==Gst.MessageType.ERROR:print(str(message.parse_error()),flush=True);loop.quit()
    if message.type==Gst.MessageType.EOS:loop.quit()
gst_bus=pipeline.get_bus();gst_bus.add_signal_watch();gst_bus.connect('message',gst_message)
GLib.timeout_add(150,poll)
signal.signal(signal.SIGTERM,lambda *_:loop.quit());signal.signal(signal.SIGINT,lambda *_:loop.quit())
pipeline.set_state(Gst.State.PLAYING)
try:loop.run()
finally:
    pipeline.set_state(Gst.State.NULL)
    bus.call_sync('org.freedesktop.portal.Desktop',session,'org.freedesktop.portal.Session','Close',None,None,Gio.DBusCallFlags.NONE,5000,None)
    os.close(fd)
