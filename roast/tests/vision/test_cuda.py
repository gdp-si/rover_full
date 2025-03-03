#!/usr/bin/env python

"""
CUDA-accelerated Computer Vision functions
"""


import unittest

import cv2
import numpy as np
from tests import SkipIfCudaNotAvailable


@SkipIfCudaNotAvailable()
class TestCuda(unittest.TestCase):
    """Test CUDA-accelerated functions"""

    def test_setup(self):
        """Setup the CUDA environment"""
        if not cv2.cuda.getCudaEnabledDeviceCount():
            self.skipTest("No CUDA-capable device is detected")

    def test_cuda_upload_download(self):
        """Test uploading and downloading data to/from GPU"""
        npMat = (np.random.random((128, 128, 3)) * 255).astype(np.uint8)
        cuMat = cv2.cuda_GpuMat()
        cuMat.upload(npMat)

        self.assertTrue(np.allclose(cuMat.download(), npMat))

    def test_cuda_upload_download_stream(self):
        """Test uploading and downloading data to/from GPU with a stream"""
        stream = cv2.cuda_Stream()
        npMat = (np.random.random((128, 128, 3)) * 255).astype(np.uint8)
        cuMat = cv2.cuda_GpuMat(128, 128, cv2.CV_8UC3)
        cuMat.upload(npMat, stream)
        npMat2 = cuMat.download(stream=stream)
        stream.waitForCompletion()
        self.assertTrue(np.allclose(npMat2, npMat))

    def test_cuda_interop(self):
        """Test interoperability between OpenCV and CUDA"""
        npMat = (np.random.random((128, 128, 3)) * 255).astype(np.uint8)
        cuMat = cv2.cuda_GpuMat()
        cuMat.upload(npMat)
        self.assertTrue(cuMat.cudaPtr() != 0)
        stream = cv2.cuda_Stream()
        self.assertTrue(stream.cudaPtr() != 0)
        asyncstream = cv2.cuda_Stream(1)  # cudaStreamNonBlocking
        self.assertTrue(asyncstream.cudaPtr() != 0)

    def test_cuda_buffer_pool(self):
        """Test the buffer pool"""
        cv2.cuda.setBufferPoolUsage(True)
        cv2.cuda.setBufferPoolConfig(cv2.cuda.getDevice(), 1024 * 1024 * 64, 2)
        stream_a = cv2.cuda.Stream()
        pool_a = cv2.cuda.BufferPool(stream_a)
        cuMat = pool_a.getBuffer(1024, 1024, cv2.CV_8UC3)
        cv2.cuda.setBufferPoolUsage(False)
        self.assertEqual(cuMat.size(), (1024, 1024))
        self.assertEqual(cuMat.type(), cv2.CV_8UC3)

    def test_cuda_release(self):
        """Test releasing a GpuMat"""
        npMat = (np.random.random((128, 128, 3)) * 255).astype(np.uint8)
        cuMat = cv2.cuda_GpuMat()
        cuMat.upload(npMat)
        cuMat.release()
        self.assertTrue(cuMat.cudaPtr() == 0)
        self.assertTrue(cuMat.step == 0)
        self.assertTrue(cuMat.size() == (0, 0))
