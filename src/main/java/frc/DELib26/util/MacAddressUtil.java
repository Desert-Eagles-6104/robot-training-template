package frc.DELib26.util;

import java.net.NetworkInterface;
import java.util.Enumeration;

public class MacAddressUtil {
	private MacAddressUtil() {}

	@SuppressWarnings("CatchAndPrintStackTrace")
	public static String getMACAddress() {
		try {
			Enumeration<NetworkInterface> networkInterface = NetworkInterface.getNetworkInterfaces();
			StringBuilder macAddress = new StringBuilder();
			while (networkInterface.hasMoreElements()) {
				NetworkInterface tempInterface =
						networkInterface.nextElement(); // Instantiates the next element in our network interface
				if (tempInterface != null) {
					byte[] mac = tempInterface.getHardwareAddress(); // Reads the MAC address from our NetworkInterface
					if (mac != null) {
						for (int i = 0; i < mac.length; i++) {
							// Formats our mac address by splitting it into two-character segments and hyphenating them
							// (unless it is the final segment)
							macAddress.append(String.format("%02X%s", mac[i], (i < mac.length - 1) ? "-" : ""));
						}
						return macAddress.toString();
					} else {
						System.err.println("Address not accessible");
					}
				} else {
					System.err.println("Network Interface for the specified address not found");
				}
			}
		} catch (Exception e) {
			e.printStackTrace();
		}

		return "";
	}
}