import pytest

from r0b0.gadgets import PiCamera
picam = PiCamera(config={
    'type':'PiCamera',
    'name':'mpi_camera',
})

class TestPiCamera():
    @pytest.mark.parametrize("shutter_speed", [1e6/15, 1e6/30, 1e6/1000])
    def test_set_shutter_speed(self, shutter_speed):
        picam.set_shutter_speed(shutter_speed)
        with picam.controls as controls:
            assert controls.ExposureTime == shutter_speed

    def test_release_shutter(self):
        # TODO - delete test_tapes directory to ensure it's empty
        # should auto-create directory if it does not exist
        save_dir = './test_tapes/'
        picam.release_shutter(save_dir=save_dir)
        assert os.path.exists(save_dir)
        n_pics = len(os.path.listdir(f'{save_dir}/*')) 
        assert  n_pics > 0
        picam.release_shutter(save_dir=save_dir)
        assert len(os.path.listdir(f'{save_dir}/*')) == n_pics+1

