import copy
from opendbc.can.parser import CANParser
from opendbc.can.packer import CANPacker


class TestCanChecksums:

  def test_honda_checksum(self):
    """Test checksums for Honda standard and extended CAN ids"""
    dbc_file = "honda_accord_2018_can_generated"
    msgs = [("LKAS_HUD", 0), ("LKAS_HUD_A", 0)]
    parser = CANParser(dbc_file, msgs, 0)
    packer = CANPacker(dbc_file)

    values = {
      'SET_ME_X41': 0x41,
      'STEERING_REQUIRED': 1,
      'SOLID_LANES': 1,
      'BEEP': 0,
    }

    # known correct checksums according to the above values
    checksum_std = [11, 10, 9, 8]
    checksum_ext = [4, 3, 2, 1]

    for std, ext in zip(checksum_std, checksum_ext, strict=True):
      msgs = [
        packer.make_can_msg("LKAS_HUD", 0, values),
        packer.make_can_msg("LKAS_HUD_A", 0, values),
      ]
      parser.update_strings([0, msgs])

      assert parser.vl['LKAS_HUD']['CHECKSUM'] == std
      assert parser.vl['LKAS_HUD_A']['CHECKSUM'] == ext

  def verify_vw_mqb_crc(self, subtests, msg_name: str, msg_addr: int, test_messages: list[int], crc_field: str = 'CHECKSUM', counter_field: str = 'COUNTER'):
    """Validates AUTOSAR E2E Profile 2 CRC calculation against recorded reference messages"""
    dbc_file = "vw_mqb_2010"
    parser = CANParser(dbc_file, [(msg_name, 0)], 0)
    packer = CANPacker(dbc_file)

    assert len(test_messages) == 16  # All counter values must be tested

    for data in test_messages:
      expected_msg = (msg_addr, data, 0)
      parser.update_strings([0, [expected_msg]])
      expected = copy.deepcopy(parser.vl[msg_name])

      modified = copy.deepcopy(expected)
      modified.pop(crc_field, None)
      modified_msg = packer.make_can_msg(msg_name, 0, modified)

      parser.update_strings([0, [modified_msg]])
      tested = parser.vl[msg_name]
      with subtests.test(counter=expected[counter_field]):
        assert tested[crc_field] == expected[crc_field]

  def test_vw_mqb_crc_LWI_01(self, subtests):
    self.verify_vw_mqb_crc(subtests, "LWI_01", 0x86, [
      b'\x6b\x00\xbd\x00\x00\x00\x00\x00',
      b'\xee\x01\x0a\x00\x00\x00\x00\x00',
      b'\xd8\x02\xa9\x00\x00\x00\x00\x00',
      b'\x03\x03\xbe\xa2\x12\x00\x00\x00',
      b'\x7b\x04\x31\x20\x03\x00\x00\x00',
      b'\x8b\x05\xe2\x85\x09\x00\x00\x00',
      b'\x63\x06\x13\x21\x00\x00\x00\x00',
      b'\x66\x07\x05\x00\x00\x00\x00\x00',
      b'\x49\x08\x0d\x00\x00\x00\x00\x00',
      b'\x5f\x09\x7e\x60\x01\x00\x00\x00',
      b'\xaf\x0a\x72\x20\x00\x00\x00\x00',
      b'\x59\x0b\x1b\x00\x00\x00\x00\x00',
      b'\xa8\x0c\x06\x00\x00\x00\x00\x00',
      b'\xbc\x0d\x72\x20\x00\x00\x00\x00',
      b'\xf9\x0e\x0f\x00\x00\x00\x00\x00',
      b'\x60\x0f\x62\xc0\x00\x00\x00\x00',
    ])

  def test_vw_mqb_crc_ACC_02(self, subtests):
    self.verify_vw_mqb_crc(subtests, "ACC_02", 780, [
      b'\x82\xf0\x3f\x00\x40\x30\x00\x40',
      b'\xe6\xf1\x3f\x00\x40\x30\x00\x40',
      b'\x4a\xf2\x3f\x00\x40\x30\x00\x40',
      b'\x2e\xf3\x3f\x00\x40\x30\x00\x40',
      b'\x3d\xf4\x3f\x00\x40\x30\x00\x40',
      b'\x59\xf5\x3f\x00\x40\x30\x00\x40',
      b'\xf5\xf6\x3f\x00\x40\x30\x00\x40',
      b'\x91\xf7\x3f\x00\x40\x30\x00\x40',
      b'\xd3\xf8\x3f\x00\x40\x30\x00\x40',
      b'\xb7\xf9\x3f\x00\x40\x30\x00\x40',
      b'\x1b\xfa\x3f\x00\x40\x30\x00\x40',
      b'\x7f\xfb\x3f\x00\x40\x30\x00\x40',
      b'\x6c\xfc\x3f\x00\x40\x30\x00\x40',
      b'\x08\xfd\x3f\x00\x40\x30\x00\x40',
      b'\xa4\xfe\x3f\x00\x40\x30\x00\x40',
      b'\xc0\xff\x3f\x00\x40\x30\x00\x40',
    ])

  def test_vw_mqb_crc_Airbag_01(self, subtests):
    self.verify_vw_mqb_crc(subtests, "Airbag_01", 0x40, [
      b'\xaf\x00\x00\x80\xc0\x00\x20\x3e',
      b'\x54\x01\x00\x80\xc0\x00\x20\x1a',
      b'\x54\x02\x00\x80\xc0\x00\x60\x00',
      b'\x31\x03\x00\x80\xc0\x00\x60\xf2',
      b'\xe0\x04\x00\x80\xc0\x00\x60\xcc',
      b'\xb3\x05\x00\x80\xc0\x00\x40\xde',
      b'\xa4\x06\x00\x80\xc0\x00\x40\x18',
      b'\x94\x07\x00\x80\xc0\x00\x20\x38',
      b'\x2d\x08\x00\x80\xc0\x00\x60\xae',
      b'\xc2\x09\x00\x80\xc0\x00\x00\x1c',
      b'\x1f\x0a\x00\x80\xc0\x00\x60\x2c',
      b'\x7f\x0b\x00\x80\xc0\x00\x00\x00',
      b'\x03\x0c\x00\x80\xc0\x00\x40\xd6',
      b'\x56\x0d\x00\x80\xc0\x00\x20\x50',
      b'\x4a\x0e\x00\x80\xc0\x00\x20\xf2',
      b'\xe5\x0f\x00\x80\xc0\x00\x40\xf6',
    ])

  def test_vw_mqb_crc_LH_EPS_03(self, subtests):
    self.verify_vw_mqb_crc(subtests, "LH_EPS_03", 0x9F, [
      b'\x11\x30\x2e\x00\x05\x1c\x80\x30',
      b'\x5b\x31\x8e\x03\x05\x53\x00\x30',
      b'\xcb\x32\xd3\x06\x05\x73\x00\x30',
      b'\xf2\x33\x28\x00\x05\x26\x00\x30',
      b'\x0b\x34\x44\x00\x05\x5b\x80\x30',
      b'\xed\x35\x80\x00\x03\x34\x00\x30',
      b'\xf0\x36\x88\x00\x05\x3d\x80\x30',
      b'\x9e\x37\x44\x03\x05\x41\x00\x30',
      b'\x68\x38\x06\x01\x05\x18\x80\x30',
      b'\x87\x39\x51\x00\x05\x11\x80\x30',
      b'\x8c\x3a\x29\x00\x05\xac\x00\x30',
      b'\x08\x3b\xbd\x00\x05\x8e\x00\x30',
      b'\xd4\x3c\x19\x00\x05\x05\x80\x30',
      b'\x29\x3d\x54\x00\x05\x5b\x00\x30',
      b'\xa1\x3e\x49\x01\x03\x04\x80\x30',
      b'\xe2\x3f\x05\x00\x05\x0a\x00\x30',
    ])

  def test_vw_mqb_crc_Getriebe_11(self, subtests):
    self.verify_vw_mqb_crc(subtests, "Getriebe_11", 0xAD, [
      b'\xf8\xe0\xbf\xff\x5f\x20\x20\x20',
      b'\xb0\xe1\xbf\xff\xc6\x98\x21\x80',
      b'\xd2\xe2\xbf\xff\x5f\x20\x20\x20',
      b'\x00\xe3\xbf\xff\xaa\x20\x20\x10',
      b'\xf1\xe4\xbf\xff\x5f\x20\x20\x20',
      b'\xc4\xe5\xbf\xff\x5f\x20\x20\x20',
      b'\xda\xe6\xbf\xff\x5f\x20\x20\x20',
      b'\x85\xe7\xbf\xff\x5f\x20\x20\x20',
      b'\x12\xe8\xbf\xff\x5f\x20\x20\x20',
      b'\x45\xe9\xbf\xff\xaa\x20\x20\x10',
      b'\x03\xea\xbf\xff\xcc\x20\x20\x10',
      b'\xfc\xeb\xbf\xff\x5f\x20\x21\x20',
      b'\xfe\xec\xbf\xff\xad\x20\x20\x10',
      b'\xbd\xed\xbf\xff\xaa\x20\x20\x10',
      b'\x67\xee\xbf\xff\xaa\x20\x20\x10',
      b'\x36\xef\xbf\xff\xaa\x20\x20\x10',
    ], counter_field="COUNTER_DISABLED")  # see opendbc#1235

  def test_vw_mqb_crc_ESP_21(self, subtests):
    self.verify_vw_mqb_crc(subtests, "ESP_21", 0xFD, [
      b'\x66\xd0\x1f\x80\x45\x05\x00\x00',
      b'\x87\xd1\x1f\x80\x52\x05\x00\x00',
      b'\xcd\xd2\x1f\x80\x50\x06\x00\x00',
      b'\xfd\xd3\x1f\x80\x35\x02\x00\x00',
      b'\xfa\xd4\x1f\x80\x22\x05\x00\x00',
      b'\xfd\xd5\x1f\x80\x84\x04\x00\x00',
      b'\x2e\xd6\x1f\x80\xf0\x03\x00\x00',
      b'\x9f\xd7\x1f\x80\x00\x00\x00\x00',
      b'\x1e\xd8\x1f\x80\xb3\x03\x00\x00',
      b'\x61\xd9\x1f\x80\x6d\x05\x00\x00',
      b'\x44\xda\x1f\x80\x47\x02\x00\x00',
      b'\x86\xdb\x1f\x80\x3a\x02\x00\x00',
      b'\x39\xdc\x1f\x80\xcb\x01\x00\x00',
      b'\x19\xdd\x1f\x80\x00\x00\x00\x00',
      b'\x8c\xde\x1f\x80\xba\x04\x00\x00',
      b'\xfb\xdf\x1f\x80\x46\x00\x00\x00',
    ])

  def test_vw_mqb_crc_ESP_02(self, subtests):
    self.verify_vw_mqb_crc(subtests, "ESP_02", 0x101, [
      b'\xf2\x00\x7e\xff\xa1\x2a\x40\x00',
      b'\xd3\x01\x7d\x00\xa2\x0c\x02\x00',
      b'\x03\x02\x7a\x06\xa2\x49\x42\x00',
      b'\xfd\x03\x70\xfb\xa1\xde\x00\x00',
      b'\x8e\x04\x7b\xf7\xa1\xd2\x01\x00',
      b'\x0f\x05\x7d\xfd\xa1\x31\x40\x00',
      b'\xb6\x06\x7d\x01\xa2\x0a\x40\x00',
      b'\xe8\x07\x7e\xfd\xa1\x12\x40\x00',
      b'\x74\x08\x7a\x01\xa2\x40\x01\x00',
      b'\xe3\x09\x81\x00\xa2\xb5\x01\x00',
      b'\xab\x0a\x74\x09\xa2\x9f\x42\x00',
      b'\xf3\x0b\x80\x12\xa2\x94\x00\x00',
      b'\x88\x0c\x7f\x07\xa2\x46\x00\x00',
      b'\x6f\x0d\x7f\xff\xa1\x53\x40\x00',
      b'\x38\x0e\x73\xd6\xa1\x6a\x40\x00',
      b'\x49\x0f\x85\x12\xa2\xf6\x01\x00',
    ])

  def test_vw_mqb_crc_ESP_05(self, subtests):
    self.verify_vw_mqb_crc(subtests, "ESP_05", 0x106, [
      b'\x90\x80\x64\x00\x00\x00\xe7\x10',
      b'\xf4\x81\x64\x00\x00\x00\xe7\x10',
      b'\x90\x82\x63\x00\x00\x00\xe8\x10',
      b'\xa0\x83\x63\x00\x00\x00\xe6\x10',
      b'\xe7\x84\x63\x00\x00\x00\xe8\x10',
      b'\x2e\x85\x78\x04\x00\x00\xea\x30',
      b'\x7b\x86\x63\x00\x00\x00\xe6\x10',
      b'\x71\x87\x79\x04\x00\x00\xd0\x30',
      b'\x50\x88\x79\x04\x00\x00\xea\x30',
      b'\x81\x89\x64\x00\x00\x00\xe1\x10',
      b'\x6a\x8a\x68\x00\x00\x04\xd0\x10',
      b'\x17\x8b\x6a\x04\x00\x00\xe6\x10',
      b'\xc7\x8c\x63\x00\x00\x00\xd1\x10',
      b'\x53\x8d\x64\x04\x00\x00\xe2\x10',
      b'\x24\x8e\x63\x00\x00\x00\xe7\x10',
      b'\x3f\x8f\x82\x04\x00\x00\xe6\x30',
    ])

  def test_vw_mqb_crc_ESP_10(self, subtests):
    self.verify_vw_mqb_crc(subtests, "ESP_10", 0x116, [
      b'\x2d\x00\xd5\x98\x9f\x26\x25\x0f',
      b'\x24\x01\x60\x63\x2c\x5e\x3b\x0f',
      b'\x08\x02\xb2\x2f\xee\x9a\x29\x0f',
      b'\x7c\x03\x17\x07\x1d\xe5\x8c\x0f',
      b'\xaa\x04\xd6\xe3\xeb\x98\xe8\x0f',
      b'\x4e\x05\xbb\xd9\x65\x43\xca\x0f',
      b'\x59\x06\x78\xbd\x25\xc6\xf2\xff',
      b'\xaf\x07\x42\x85\x53\xbe\xbe\x0f',
      b'\x2a\x08\xa6\xcd\x95\x8c\x12\x0f',
      b'\xce\x09\x6e\x17\x6d\x1b\x2f\x0f',
      b'\x60\x0a\xd3\xe6\x3a\x8d\xf0\x0f',
      b'\xc5\x0b\xfc\x69\x57\x50\x21\x0f',
      b'\x70\x0c\xde\xf3\x9d\xe9\x6b\xff',
      b'\x62\x0d\xc4\x1a\xdb\x61\x7a\x0f',
      b'\x76\x0e\x79\x69\xe3\x32\x67\x0f',
      b'\x15\x0f\x51\x59\x56\x35\xb1\x0f',
    ])

  def test_vw_mqb_crc_ACC_10(self, subtests):
    self.verify_vw_mqb_crc(subtests, "ACC_10", 0x117, [
      b'\x9b\x00\x00\x40\x68\x00\x00\xff',
      b'\xff\x01\x00\x40\x68\x00\x00\xff',
      b'\x53\x02\x00\x40\x68\x00\x00\xff',
      b'\x37\x03\x00\x40\x68\x00\x00\xff',
      b'\x24\x04\x00\x40\x68\x00\x00\xff',
      b'\x40\x05\x00\x40\x68\x00\x00\xff',
      b'\xec\x06\x00\x40\x68\x00\x00\xff',
      b'\x88\x07\x00\x40\x68\x00\x00\xff',
      b'\xca\x08\x00\x40\x68\x00\x00\xff',
      b'\xae\x09\x00\x40\x68\x00\x00\xff',
      b'\x02\x0a\x00\x40\x68\x00\x00\xff',
      b'\x66\x0b\x00\x40\x68\x00\x00\xff',
      b'\x75\x0c\x00\x40\x68\x00\x00\xff',
      b'\x11\x0d\x00\x40\x68\x00\x00\xff',
      b'\xbd\x0e\x00\x40\x68\x00\x00\xff',
      b'\xd9\x0f\x00\x40\x68\x00\x00\xff',
    ])

  def test_vw_mqb_crc_TSK_06(self, subtests):
    self.verify_vw_mqb_crc(subtests, "TSK_06", 0x120, [
      b'\xc1\x00\x00\x02\x00\x08\xff\x21',
      b'\x34\x01\x00\x02\x00\x08\xff\x21',
      b'\xcc\x02\x00\x02\x00\x08\xff\x21',
      b'\x1e\x03\x00\x02\x00\x08\xff\x21',
      b'\x48\x04\x00\x02\x00\x08\xff\x21',
      b'\x4a\x05\x00\x02\x00\x08\xff\x21',
      b'\xa5\x06\x00\x02\x00\x08\xff\x21',
      b'\xa7\x07\x00\x02\x00\x08\xff\x21',
      b'\xfe\x08\x00\x02\x00\x08\xff\x21',
      b'\xa8\x09\x00\x02\x00\x08\xff\x21',
      b'\x73\x0a\x00\x02\x00\x08\xff\x21',
      b'\xdf\x0b\x00\x02\x00\x08\xff\x21',
      b'\x05\x0c\x00\x02\x00\x08\xff\x21',
      b'\xb5\x0d\x00\x02\x00\x08\xff\x21',
      b'\xde\x0e\x00\x02\x00\x08\xff\x21',
      b'\x0b\x0f\x00\x02\x00\x08\xff\x21',
    ])

  def test_vw_mqb_crc_Motor_20(self, subtests):
    self.verify_vw_mqb_crc(subtests, "Motor_20", 0x121, [
      b'\xb9\x00\x00\xc0\x39\x46\x7e\xfe',
      b'\x85\x31\x20\x00\x1a\x46\x7e\xfe',
      b'\xc7\x12\x00\x40\x1a\x46\x7e\xfe',
      b'\x53\x93\x00\x00\x19\x46\x7e\xfe',
      b'\xa4\x34\x00\x80\x1a\x46\x7e\xfe',
      b'\x0e\x55\x20\x60\x18\x46\x7e\xfe',
      b'\x3f\x06\x00\xc0\x37\x4c\x7e\xfe',
      b'\x0c\x07\x00\x40\x39\x46\x7e\xfe',
      b'\x2a\x08\x00\x00\x3a\x46\x7e\xfe',
      b'\x7f\x49\x20\x80\x1a\x46\x7e\xfe',
      b'\x2f\x0a\x00\xc0\x39\x46\x7e\xfe',
      b'\x70\xbb\x00\x00\x17\x46\x7e\xfe',
      b'\x06\x0c\x00\x00\x39\x46\x7e\xfe',
      b'\x4b\x9d\x20\xe0\x16\x4c\x7e\xfe',
      b'\x73\xfe\x00\x40\x16\x46\x7e\xfe',
      b'\xaf\x0f\x20\x80\x39\x4c\x7e\xfe',
    ])

  def test_vw_mqb_crc_ACC_06(self, subtests):
    self.verify_vw_mqb_crc(subtests, "ACC_06", 0x122, [
      b'\x14\x80\x00\xfe\x07\x00\x00\x18',
      b'\x9f\x81\x00\xfe\x07\x00\x00\x18',
      b'\x0a\x82\x00\xfe\x07\x00\x00\x28',
      b'\x40\x83\x00\xfe\x07\x00\x00\x18',
      b'\x2d\x84\x00\xfe\x07\x00\x00\x28',
      b'\xdb\x85\x00\xfe\x07\x00\x00\x18',
      b'\x4d\x86\x00\xfe\x07\x00\x00\x28',
      b'\x35\x87\x00\xfe\x07\x00\x00\x18',
      b'\x23\x88\x00\xfe\x07\x00\x00\x28',
      b'\x4a\x89\x00\xfe\x07\x00\x00\x28',
      b'\xe1\x8a\x00\xfe\x07\x00\x00\x28',
      b'\x30\x8b\x00\xfe\x07\x00\x00\x28',
      b'\x60\x8c\x00\xfe\x07\x00\x00\x28',
      b'\x0d\x8d\x00\xfe\x07\x00\x00\x18',
      b'\x8c\x8e\x00\xfe\x07\x00\x00\x18',
      b'\x6f\x8f\x00\xfe\x07\x00\x00\x28',
    ])

  def test_vw_mqb_crc_HCA_01(self, subtests):
    self.verify_vw_mqb_crc(subtests, "HCA_01", 0x126, [
      b'\x00\x30\x0d\xc0\x05\xfe\x07\x00',
      b'\x3e\x31\x54\xc0\x05\xfe\x07\x00',
      b'\xa7\x32\xbb\x40\x05\xfe\x07\x00',
      b'\x96\x33\x29\xc0\x05\xfe\x07\x00',
      b'\x5f\x34\x00\x00\x03\xfe\x07\x00',
      b'\x3b\x35\xae\x40\x05\xfe\x07\x00',
      b'\xc7\x36\x7a\x40\x05\xfe\x07\x00',
      b'\x6f\x37\x76\x40\x05\xfe\x07\x00',
      b'\xb1\x38\x00\x00\x03\xfe\x07\x00',
      b'\xd5\x39\x00\x00\x03\xfe\x07\x00',
      b'\xba\x3a\x69\xc0\x05\xfe\x07\x00',
      b'\x65\x3b\x10\x40\x05\xfe\x07\x00',
      b'\x49\x3c\x72\xc0\x05\xfe\x07\x00',
      b'\xc6\x3d\xdf\x40\x05\xfe\x07\x00',
      b'\x1d\x3e\x2c\xc1\x05\xfe\x07\x00',
      b'\x9b\x3f\x20\x40\x05\xfe\x07\x00',
    ])

  def test_vw_mqb_crc_GRA_ACC_01(self, subtests):
    self.verify_vw_mqb_crc(subtests, "GRA_ACC_01", 0x12B, [
      b'\x86\x40\x80\x2a\x00\x00\x00\x00',
      b'\xf4\x41\x80\x2a\x00\x00\x00\x00',
      b'\x50\x42\x80\x2a\x00\x00\x00\x00',
      b'\x08\x43\x80\x2a\x00\x00\x00\x00',
      b'\x88\x44\x80\x2a\x00\x00\x00\x00',
      b'\x2d\x45\x80\x2a\x00\x00\x00\x00',
      b'\x34\x46\x80\x2a\x00\x00\x00\x00',
      b'\x11\x47\x80\x2a\x00\x00\x00\x00',
      b'\xc4\x48\x80\x2a\x00\x00\x00\x00',
      b'\xcc\x49\x80\x2a\x00\x00\x00\x00',
      b'\xdc\x4a\x80\x2a\x00\x00\x00\x00',
      b'\x79\x4b\x80\x2a\x00\x00\x00\x00',
      b'\x3c\x4c\x80\x2a\x00\x00\x00\x00',
      b'\x68\x4d\x80\x2a\x00\x00\x00\x00',
      b'\x27\x4e\x80\x2a\x00\x00\x00\x00',
      b'\x0d\x4f\x80\x2a\x00\x00\x00\x00',
    ])

  def test_vw_mqb_crc_ACC_07(self, subtests):
    self.verify_vw_mqb_crc(subtests, "ACC_07", 0x12E, [
      b'\xac\xe0\x7f\x00\xfe\x00\xc0\xff',
      b'\xa2\xe1\x7f\x00\xfe\x00\xc0\xff',
      b'\x6b\xe2\x7f\x00\xfe\x00\xc0\xff',
      b'\xf2\xe3\x7f\x00\xfe\x00\xc0\xff',
      b'\xd5\xe4\x7f\x00\xfe\x00\xc0\xff',
      b'\x35\xe5\x7f\x00\xfe\x00\xc0\xff',
      b'\x7f\xe6\x7f\x00\xfe\x00\xc0\xff',
      b'\x6c\xe7\x7f\x00\xfe\x00\xc0\xff',
      b'\x05\xe8\x7f\x00\xfe\x00\xc0\xff',
      b'\x79\xe9\x7f\x00\xfe\x00\xc0\xff',
      b'\x25\xea\x7f\x00\xfe\x00\xc0\xff',
      b'\xd1\xeb\x7f\x00\xfe\x00\xc0\xff',
      b'\x72\xec\x7f\x00\xfe\x00\xc0\xff',
      b'\x58\xed\x7f\x00\xfe\x00\xc0\xff',
      b'\x82\xee\x7f\x00\xfe\x00\xc0\xff',
      b'\x85\xef\x7f\x00\xfe\x00\xc0\xff',
    ])

  def test_vw_mqb_crc_Motor_EV_01(self, subtests):
    self.verify_vw_mqb_crc(subtests, "Motor_EV_01", 0x187, [
      b'\x70\x80\x15\x00\x00\x00\x00\xF0',
      b'\x07\x81\x15\x00\x00\x00\x00\xF0',
      b'\x7A\x82\x15\x00\x00\x00\x00\xF0',
      b'\x26\x83\x15\x00\x00\x00\x00\xF0',
      b'\xBE\x84\x15\x00\x00\x00\x00\xF0',
      b'\x5A\x85\x15\x00\x00\x00\x00\xF0',
      b'\xFC\x86\x15\x00\x00\x00\x00\xF0',
      b'\x9E\x87\x15\x00\x00\x00\x00\xF0',
      b'\xAF\x88\x15\x00\x00\x00\x00\xF0',
      b'\x35\x89\x15\x00\x00\x00\x00\xF0',
      b'\xC5\x8A\x15\x00\x00\x00\x00\xF0',
      b'\x11\x8B\x15\x00\x00\x00\x00\xF0',
      b'\xD0\x8C\x15\x00\x00\x00\x00\xF0',
      b'\xE8\x8D\x15\x00\x00\x00\x00\xF0',
      b'\xF5\x8E\x15\x00\x00\x00\x00\xF0',
      b'\x00\x8F\x15\x00\x00\x00\x00\xF0',
    ])

  def test_vw_mqb_crc_ESP_33(self, subtests):
    self.verify_vw_mqb_crc(subtests, "ESP_33", 0x1AB, [
      b'\x64\x00\x80\x02\x00\x00\x00\x00',
      b'\x19\x01\x00\x00\x00\x00\x00\x00',
      b'\xfc\x02\x00\x10\x01\x00\x00\x00',
      b'\x8b\x03\x80\x02\x00\x00\x00\x00',
      b'\xa4\x04\x00\x10\x01\x00\x00\x00',
      b'\x97\x05\x00\x02\x00\x00\x01\x00',
      b'\xd5\x06\x80\x02\x00\x00\x01\x00',
      b'\xa0\x07\x80\x02\x00\x00\x01\x00',
      b'\x89\x08\x00\x00\x00\x00\x00\x00',
      b'\xe3\x09\x00\x00\x00\x00\x00\x00',
      b'\x0e\x0a\x00\x00\x00\x00\x00\x00',
      b'\x90\x0b\x00\x00\x00\x00\x00\x00',
      b'\x32\x0c\x00\x10\x01\x00\x00\x00',
      b'\x30\x0d\x00\x00\x00\x00\x00\x00',
      b'\xc2\x0e\x00\x10\x01\x00\x00\x00',
      b'\x68\x0f\x80\x02\x00\x00\x00\x00',
    ])

  def test_vw_mqb_crc_SWA_01(self, subtests):
    self.verify_vw_mqb_crc(subtests, "SWA_01", 0x30F, [
      b'\x10\x00\x10\x00\x00\x00\x00\x00',
      b'\x74\x01\x10\x00\x00\x00\x00\x00',
      b'\xD8\x02\x10\x00\x00\x00\x00\x00',
      b'\xBC\x03\x10\x00\x00\x00\x00\x00',
      b'\xAF\x04\x10\x00\x00\x00\x00\x00',
      b'\xCB\x05\x10\x00\x00\x00\x00\x00',
      b'\x67\x06\x10\x00\x00\x00\x00\x00',
      b'\x03\x07\x10\x00\x00\x00\x00\x00',
      b'\x41\x08\x10\x00\x00\x00\x00\x00',
      b'\x25\x09\x10\x00\x00\x00\x00\x00',
      b'\x89\x0A\x10\x00\x00\x00\x00\x00',
      b'\xED\x0B\x10\x00\x00\x00\x00\x00',
      b'\xFE\x0C\x10\x00\x00\x00\x00\x00',
      b'\x9A\x0D\x10\x00\x00\x00\x00\x00',
      b'\x36\x0E\x10\x00\x00\x00\x00\x00',
      b'\x52\x0F\x10\x00\x00\x00\x00\x00',
    ])

  def test_vw_mqb_crc_ACC_04(self, subtests):
    self.verify_vw_mqb_crc(subtests, "ACC_04", 0x324, [
      b'\xba\x00\x00\x00\x00\x00\x00\x10',
      b'\xde\x01\x00\x00\x00\x00\x00\x10',
      b'\x72\x02\x00\x00\x00\x00\x00\x10',
      b'\x16\x03\x00\x00\x00\x00\x00\x10',
      b'\x05\x04\x00\x00\x00\x00\x00\x10',
      b'\x44\x05\x00\x00\x00\x00\x00\x00',
      b'\xe8\x06\x00\x00\x00\x00\x00\x00',
      b'\xa9\x07\x00\x00\x00\x00\x00\x10',
      b'\xeb\x08\x00\x00\x00\x00\x00\x10',
      b'\x8f\x09\x00\x00\x00\x00\x00\x10',
      b'\x06\x0a\x00\x00\x00\x00\x00\x00',
      b'\x47\x0b\x00\x00\x00\x00\x00\x10',
      b'\x71\x0c\x00\x00\x00\x00\x00\x00',
      b'\x15\x0d\x00\x00\x00\x00\x00\x00',
      b'\xb9\x0e\x00\x00\x00\x00\x00\x00',
      b'\xdd\x0f\x00\x00\x00\x00\x00\x00',
    ])

  def test_vw_mqb_crc_Klemmen_Status_01(self, subtests):
    self.verify_vw_mqb_crc(subtests, "Klemmen_Status_01", 0x3C0, [
      b'\x74\x00\x03\x00',
      b'\xc1\x01\x03\x00',
      b'\x31\x02\x03\x00',
      b'\x84\x03\x03\x00',
      b'\xfe\x04\x03\x00',
      b'\x4b\x05\x03\x00',
      b'\xbb\x06\x03\x00',
      b'\x0e\x07\x03\x00',
      b'\x4f\x08\x03\x00',
      b'\xfa\x09\x03\x00',
      b'\x0a\x0a\x03\x00',
      b'\xbf\x0b\x03\x00',
      b'\xc5\x0c\x03\x00',
      b'\x70\x0d\x03\x00',
      b'\x80\x0e\x03\x00',
      b'\x35\x0f\x03\x00',
    ])

  def test_vw_mqb_crc_Licht_Anf_01(self, subtests):
    self.verify_vw_mqb_crc(subtests, "Licht_Anf_01", 0x3D5, [
      b'\xc8\x00\x00\x04\x00\x00\x00\x00',
      b'\x9f\x01\x00\x04\x00\x00\x00\x00',
      b'\x5e\x02\x00\x04\x00\x00\x00\x00',
      b'\x52\x03\x00\x04\x00\x00\x00\x00',
      b'\xf2\x04\x00\x04\x00\x00\x00\x00',
      b'\x79\x05\x00\x04\x00\x00\x00\x00',
      b'\xe6\x06\x00\x04\x00\x00\x00\x00',
      b'\xfd\x07\x00\x04\x00\x00\x00\x00',
      b'\xf8\x08\x00\x04\x00\x00\x00\x00',
      b'\xc6\x09\x00\x04\x00\x00\x00\x00',
      b'\xf5\x0a\x00\x04\x00\x00\x00\x00',
      b'\x1a\x0b\x00\x04\x00\x00\x00\x00',
      b'\x65\x0c\x00\x04\x00\x00\x00\x00',
      b'\x41\x0d\x00\x04\x00\x00\x00\x00',
      b'\x7f\x0e\x00\x04\x00\x00\x00\x00',
      b'\x98\x0f\x00\x04\x00\x00\x00\x00',
    ])

  def test_vw_mqb_crc_ESP_20(self, subtests):
    self.verify_vw_mqb_crc(subtests, "ESP_20", 0x65D, [
      b'\x98\x30\x2b\x10\x00\x00\x22\x81',
      b'\xc8\x31\x2b\x10\x00\x00\x22\x81',
      b'\x9d\x32\x2b\x10\x00\x00\x22\x81',
      b'\x1f\x33\x2b\x10\x00\x00\x22\x81',
      b'\x6e\x34\x2b\x10\x00\x00\x22\x81',
      b'\x61\x35\x2b\x10\x00\x00\x22\x81',
      b'\x6f\x36\x2b\x10\x00\x00\x22\x81',
      b'\xe5\x37\x2b\x10\x00\x00\x22\x81',
      b'\xf8\x38\x2b\x10\x00\x00\x22\x81',
      b'\xe1\x39\x2b\x10\x00\x00\x22\x81',
      b'\xaa\x3a\x2b\x10\x00\x00\x22\x81',
      b'\xe6\x3b\x2b\x10\x00\x00\x22\x81',
      b'\xef\x3c\x2b\x10\x00\x00\x22\x81',
      b'\xbb\x3d\x2b\x10\x00\x00\x22\x81',
      b'\x9b\x3e\x2b\x10\x00\x00\x22\x81',
      b'\x72\x3f\x2b\x10\x00\x00\x22\x81',
    ])
