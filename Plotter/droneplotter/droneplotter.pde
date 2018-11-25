import java.net.DatagramSocket;
import java.net.DatagramPacket;
import java.net.InetAddress;
import java.net.SocketException;
import java.nio.ByteBuffer;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.nio.ByteOrder;
import java.nio.FloatBuffer;

public class FloatReceiver extends Thread {

  private DatagramSocket socket;
  private boolean running;
  private ByteBuffer buf = ByteBuffer.allocate(256);

  private Queue<Float> fbuf = new ArrayBlockingQueue<Float>(32);

  public FloatReceiver() throws SocketException {
    buf.order(ByteOrder.LITTLE_ENDIAN);
    socket = new DatagramSocket(4445);
  }

  public void run() {
    try {
      running = true;

      while (running) {
        buf.clear();
        DatagramPacket packet 
          = new DatagramPacket(buf.array(), buf.capacity());
        socket.receive(packet);

        InetAddress address = packet.getAddress();
        int port = packet.getPort();
        
        FloatBuffer floatbuf = buf.asFloatBuffer();

        System.out.println("Received UDP packet from " + address.toString() + ":" + port);
        
        for (int i = 0; i < packet.getLength(); i += 4) {
          float f = floatbuf.get();
          addToQueue(f);
          System.out.println(f);
        }
      }
      socket.close();
    } 
    catch (IOException e) {
      System.err.println("Error in UDP thread: " + e.getMessage());
    }
  }

  public synchronized void addToQueue(float f) {
    fbuf.add(f);
  }

  public synchronized float getFromQueue() {
    if (getQueueSize() > 0)
      return fbuf.remove();
    else 
    return Float.NaN;
  }

  public synchronized int getQueueSize() {
    return fbuf.size();
  }
};


// A PShape object
PShape path;

float[] buffer;
int index = 0;
int viewportwidth;

final color bg = color(51);

FloatReceiver floatreceiver;

void setup() {
  size(192, 100, P2D);

  viewportwidth = width;
  buffer = new float[viewportwidth];
  frameRate(60);

  try {
    floatreceiver = new FloatReceiver();
    floatreceiver.start();
  } 
  catch (SocketException e) {
    System.err.println("Error in creating UDP socket: " + e.getMessage());
  }
}

boolean started = false;
int prebuffer = 0;

void draw() {
  if (!started)
    if (floatreceiver.getQueueSize() > prebuffer)
      started = true;
    else
      return;

  // println(frameRate);
  background(bg);
    
  buffer[index] = floatreceiver.getFromQueue();

  // Create the shape
  path = createShape();
  path.beginShape();
  // Set fill and stroke
  path.noFill();
  path.stroke(255, 0, 0);
  path.strokeWeight(1);

  int i = index;
  for (int ctr = viewportwidth; ctr--> 0; ) {
    if (Float.isNaN(buffer[i])) {
      path.stroke(bg);
      path.vertex(ctr, 0);
      path.stroke(255, 0, 0);
    } else {
      path.vertex(ctr, buffer[i]);
    }
    if (i == 0)
      i = buffer.length;
    --i;
  }
  // The path is complete
  path.endShape();  

  shape(path);

  index++;
  if (index == buffer.length)
    index = 0;
}
