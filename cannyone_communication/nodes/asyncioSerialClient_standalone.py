#!/usr/bin/env python3

"""Testing Python Script for receiving Asyncio Serial data from MCU """

import asyncio
import serial_asyncio


class Output(asyncio.Protocol):
    def connection_made(self, transport):
        """Store the serial transport and prepare to receive data.
        """
        self.transport = transport
        self.buf = bytes()
        print('commROSMCU Reader connection created', transport)
        transport.serial.rts = False
        transport.write(b'hello world\n')

    def data_received(self, data):
        """Store characters until a newline is received.
        """
        self.buf += data
        if b'\n' in self.buf:
            lines = self.buf.split(b'\n')
            self.buf = lines[-1]
            for line in lines[:-1]:
                print(f'Reader received: {line.decode()}')

    def connection_lost(self, exc):
        print('commROSMCU Reader port closed')
        self.transport.close()
        asyncio.get_event_loop().stop()


if __name__ == '__main__':

    loop = asyncio.get_event_loop()
    coro = serial_asyncio.create_serial_connection(loop, Output, '/dev/ttyACM0', baudrate=9600)

    # asyncio.ensure_future(coro)
    try: 
        loop.run_until_complete(coro)
        loop.run_forever()
    except Exception as err:
        print (err)
    finally:
        loop.close()