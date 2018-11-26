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
  private ByteBuffer buf = ByteBuffer.allocate(1024);

  private Queue<Float> floatqueue = new ArrayBlockingQueue<Float>(256);

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

        buf.limit(packet.getLength());

        InetAddress address = packet.getAddress();
        int port = packet.getPort();

        FloatBuffer floatbuf = buf.asFloatBuffer();

        System.out.println("Received UDP packet from " + address.toString() + ":" + port);

        // for (int i = 0; i < packet.getLength(); i += 4) {
        while (floatbuf.hasRemaining()) {
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
    floatqueue.add(f);
  }

  public synchronized float getFromQueue() {
    if (getQueueSize() > 0)
      return floatqueue.remove();
    return Float.NaN;  
  }

  private synchronized int getQueueSize() {
    return floatqueue.size();
  }
};


// A PShape object
PShape plot;

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

void draw() {
  // clear the display
  background(bg);

  // add a new value to the buffer
  buffer[index] = floatreceiver.getFromQueue();

  plot = createShape();
  plot.beginShape();
  plot.noFill();
  plot.stroke(255, 0, 0);
  plot.strokeWeight(1);

  int i = index;
  float lastfinitevalue = 0;
  for (int ctr = viewportwidth; ctr--> 0; ) {
    if (Float.isNaN(buffer[i])) {
      plot.stroke(bg);
      plot.vertex(ctr, lastfinitevalue);
      plot.stroke(255, 0, 0);
    } else {
      plot.vertex(ctr, buffer[i]);
      lastfinitevalue = buffer[i];
    }
    if (i == 0)
      i = buffer.length;
    --i;
  }
  plot.endShape();  

  shape(plot);

  index++;
  if (index == buffer.length)
    index = 0;
}
