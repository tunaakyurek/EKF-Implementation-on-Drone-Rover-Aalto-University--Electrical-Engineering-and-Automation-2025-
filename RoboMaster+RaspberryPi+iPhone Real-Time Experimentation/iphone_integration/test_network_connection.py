#!/usr/bin/env python3
"""
Simple network test to verify iPhone can connect to Raspberry Pi
Run this on the Raspberry Pi to test UDP connectivity
"""

import socket
import time
import sys

def test_udp_connection(port=5555):
    """Test UDP connection and print detailed info"""
    
    print(f"🔍 Network Connection Test")
    print("=" * 50)
    
    # Create UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.settimeout(1.0)
    
    try:
        # Bind to all interfaces
        sock.bind(('', port))
        print(f"✅ Successfully bound to port {port}")
        
        # Get local IP addresses
        hostname = socket.gethostname()
        print(f"📡 Hostname: {hostname}")
        
        # Try to get local IP
        try:
            # Connect to a remote server to find local IP
            test_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            test_sock.connect(("8.8.8.8", 80))
            local_ip = test_sock.getsockname()[0]
            test_sock.close()
            print(f"📍 Detected local IP: {local_ip}")
        except:
            print("⚠️  Could not detect local IP automatically")
        
        print(f"\n🎯 Configure your iPhone app to send data to:")
        print(f"   IP: {local_ip if 'local_ip' in locals() else '172.20.10.12'}")
        print(f"   Port: {port}")
        print(f"   Protocol: UDP")
        
        print(f"\n⏳ Listening for data on port {port}...")
        print("   Send data from iPhone now...")
        print("   Press Ctrl+C to stop")
        
        packet_count = 0
        start_time = time.time()
        
        while True:
            try:
                data, addr = sock.recvfrom(8192)
                packet_count += 1
                elapsed = time.time() - start_time
                
                print(f"\n📦 Packet #{packet_count} (t={elapsed:.1f}s)")
                print(f"   From: {addr[0]}:{addr[1]}")
                print(f"   Size: {len(data)} bytes")
                print(f"   Preview: {data[:100]}...")
                
                # Try to decode as text
                try:
                    text = data.decode('utf-8', errors='ignore')
                    if text.startswith('{'):
                        print(f"   Type: JSON data ✅")
                        # Count JSON fields
                        import json
                        try:
                            parsed = json.loads(text)
                            print(f"   JSON fields: {len(parsed)}")
                        except:
                            print(f"   JSON: Invalid format")
                    else:
                        print(f"   Type: Text data")
                except:
                    print(f"   Type: Binary data")
                
            except socket.timeout:
                # Print a dot every second to show we're still listening
                if int(time.time()) % 5 == 0:
                    print(".", end="", flush=True)
                continue
                
    except KeyboardInterrupt:
        print(f"\n\n🛑 Stopped by user")
        if packet_count > 0:
            elapsed = time.time() - start_time
            print(f"📊 Statistics:")
            print(f"   Total packets: {packet_count}")
            print(f"   Duration: {elapsed:.1f}s")
            print(f"   Average rate: {packet_count/elapsed:.1f} packets/s")
        else:
            print("❌ No packets received")
            print("\n🔧 Troubleshooting:")
            print("   1. Check iPhone app is sending to correct IP:port")
            print("   2. Verify iPhone and Pi are on same network")
            print("   3. Check firewall settings on Pi")
            print("   4. Try sending test data with: nc -u 172.20.10.12 5555")
    
    except Exception as e:
        print(f"❌ Error: {e}")
    
    finally:
        sock.close()
        print("🔚 Socket closed")

if __name__ == "__main__":
    import argparse
    
    parser = argparse.ArgumentParser(description='Test UDP network connection')
    parser.add_argument('--port', type=int, default=5555, help='UDP port to listen on')
    
    args = parser.parse_args()
    test_udp_connection(args.port)
