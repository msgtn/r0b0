"""
Certificate management utilities for HTTPS support.
Handles automatic installation of mkcert and generation of locally-trusted certificates.
"""

import os
import subprocess
import sys
import shutil
import platform
from pathlib import Path
from datetime import datetime, timedelta
import logging

logger = logging.getLogger(__name__)


class CertificateManager:
    """Manages SSL/TLS certificates for local HTTPS development."""
    
    def __init__(self, cert_path: Path, key_path: Path):
        """
        Initialize certificate manager.
        
        Args:
            cert_path: Path to certificate file (e.g., .keys/csr.pem)
            key_path: Path to private key file (e.g., .keys/key.pem)
        """
        self.cert_path = Path(cert_path)
        self.key_path = Path(key_path)
        self.keys_dir = self.cert_path.parent
        
    def ensure_certificates(self) -> tuple[Path, Path]:
        """
        Ensure valid certificates exist. Auto-generates if missing.
        
        Returns:
            tuple: (cert_path, key_path)
            
        Raises:
            RuntimeError: If certificate generation fails
        """
        # Check if certificates already exist and are valid
        if self._certificates_exist():
            logger.info("✓ SSL certificates found")
            
            # Check expiry
            days_remaining = self._check_certificate_expiry()
            if days_remaining is not None:
                if days_remaining < 30:
                    logger.warning(
                        f"⚠ Certificate expires in {days_remaining} days! "
                        "Consider regenerating certificates."
                    )
                elif days_remaining < 7:
                    logger.error(
                        f"⚠⚠ CRITICAL: Certificate expires in {days_remaining} days! "
                        "Regenerating certificates..."
                    )
                    self._generate_certificates()
                else:
                    logger.info(f"Certificate valid for {days_remaining} more days")
        else:
            logger.info("No certificates found. Generating new certificates...")
            self._generate_certificates()
            
        return self.cert_path, self.key_path
    
    def _certificates_exist(self) -> bool:
        """Check if both certificate files exist."""
        return self.cert_path.exists() and self.key_path.exists()
    
    def _check_certificate_expiry(self) -> int | None:
        """
        Check certificate expiration date.
        
        Returns:
            int: Days until expiry, or None if unable to check
        """
        try:
            # Try using cryptography library if available
            try:
                from cryptography import x509
                from cryptography.hazmat.backends import default_backend
                
                with open(self.cert_path, 'rb') as f:
                    cert = x509.load_pem_x509_certificate(f.read(), default_backend())
                
                days_until_expiry = (cert.not_valid_after - datetime.now()).days
                return days_until_expiry
                
            except ImportError:
                # Fallback: use OpenSSL command
                result = subprocess.run(
                    ['openssl', 'x509', '-enddate', '-noout', '-in', str(self.cert_path)],
                    capture_output=True,
                    text=True,
                    timeout=5
                )
                
                if result.returncode == 0:
                    # Parse output: "notAfter=Oct  3 12:00:00 2026 GMT"
                    expiry_str = result.stdout.strip().split('=')[1]
                    expiry_date = datetime.strptime(expiry_str, '%b %d %H:%M:%S %Y %Z')
                    days_until_expiry = (expiry_date - datetime.now()).days
                    return days_until_expiry
                    
        except Exception as e:
            logger.debug(f"Could not check certificate expiry: {e}")
            
        return None
    
    def _generate_certificates(self):
        """Generate SSL certificates using mkcert or fallback to self-signed."""
        # Ensure .keys directory exists
        self.keys_dir.mkdir(parents=True, exist_ok=True)
        
        # Try mkcert first (trusted certificates)
        if self._try_mkcert_generation():
            logger.info("✓ Generated locally-trusted certificates with mkcert")
            logger.info("  Your browser will NOT show security warnings!")
            return
        
        # Fallback to self-signed certificates
        logger.warning(
            "mkcert not available. Generating self-signed certificates.\n"
            "  Your browser will show security warnings that you must accept.\n"
            "  For better experience, install mkcert: https://github.com/FiloSottile/mkcert"
        )
        self._generate_self_signed()
        logger.info("✓ Generated self-signed certificates")
        logger.warning(
            "  ⚠ Browser will show 'Your connection is not private' warning\n"
            "  ⚠ Click 'Advanced' → 'Proceed to localhost (unsafe)' to continue"
        )
    
    def _try_mkcert_generation(self) -> bool:
        """
        Try to generate certificates using mkcert.
        Auto-installs mkcert if not present.
        
        Returns:
            bool: True if successful, False otherwise
        """
        try:
            # Check if mkcert is installed
            if not self._is_mkcert_installed():
                logger.info("mkcert not found. Attempting to install...")
                if not self._install_mkcert():
                    return False
            
            # Install local CA (if not already installed)
            logger.info("Setting up local certificate authority...")
            result = subprocess.run(
                ['mkcert', '-install'],
                capture_output=True,
                text=True,
                timeout=30
            )
            
            if result.returncode != 0:
                logger.warning(f"Could not install mkcert CA: {result.stderr}")
                return False
            
            # Generate certificates for localhost and common local addresses
            logger.info("Generating trusted certificates...")
            cmd = [
                'mkcert',
                '-key-file', str(self.key_path),
                '-cert-file', str(self.cert_path),
                'localhost',
                '127.0.0.1',
                '::1',
                '*.local',
                # Add common local network ranges
                '192.168.1.*',
                '192.168.0.*',
                '10.0.0.*',
            ]
            
            result = subprocess.run(
                cmd,
                capture_output=True,
                text=True,
                timeout=30,
                cwd=str(self.keys_dir)
            )
            
            if result.returncode == 0:
                logger.info(f"mkcert output: {result.stdout}")
                return True
            else:
                logger.warning(f"mkcert generation failed: {result.stderr}")
                return False
                
        except Exception as e:
            logger.warning(f"mkcert generation failed: {e}")
            return False
    
    def _is_mkcert_installed(self) -> bool:
        """Check if mkcert is installed and accessible."""
        return shutil.which('mkcert') is not None
    
    def _install_mkcert(self) -> bool:
        """
        Attempt to install mkcert based on the operating system.
        
        Returns:
            bool: True if installation successful, False otherwise
        """
        system = platform.system()
        
        try:
            if system == 'Windows':
                return self._install_mkcert_windows()
            elif system == 'Darwin':  # macOS
                return self._install_mkcert_macos()
            elif system == 'Linux':
                return self._install_mkcert_linux()
            else:
                logger.error(f"Unsupported OS: {system}")
                return False
                
        except Exception as e:
            logger.error(f"Failed to install mkcert: {e}")
            return False
    
    def _install_mkcert_windows(self) -> bool:
        """Install mkcert on Windows using winget (preinstalled on most Windows machines)."""
        # Try winget first (preinstalled on Windows 10/11)
        if shutil.which('winget'):
            logger.info("Installing mkcert via winget...")
            result = subprocess.run(
                ['winget', 'install', '-e', '--id', 'FiloSottile.mkcert'],
                capture_output=True,
                text=True,
                timeout=300  # 5 minutes
            )
            
            if result.returncode == 0:
                logger.info("✓ mkcert installed via winget")
                # Refresh PATH
                self._refresh_windows_path()
                return True
        
        # Try Chocolatey as fallback
        if shutil.which('choco'):
            logger.info("Installing mkcert via Chocolatey...")
            result = subprocess.run(
                ['choco', 'install', 'mkcert', '-y'],
                capture_output=True,
                text=True,
                timeout=300
            )
            
            if result.returncode == 0:
                logger.info("✓ mkcert installed via Chocolatey")
                self._refresh_windows_path()
                return True
        
        # Try Scoop as second fallback
        if shutil.which('scoop'):
            logger.info("Installing mkcert via Scoop...")
            result = subprocess.run(
                ['scoop', 'install', 'mkcert'],
                capture_output=True,
                text=True,
                timeout=300
            )
            
            if result.returncode == 0:
                logger.info("✓ mkcert installed via Scoop")
                return True
        
        # Manual download fallback
        logger.warning(
            "Could not install mkcert automatically.\n"
            "Please install mkcert manually:\n"
            "  1. Run: winget install -e --id FiloSottile.mkcert\n"
            "  2. Or download from: https://github.com/FiloSottile/mkcert/releases\n"
            "  3. Rename to 'mkcert.exe' and add to PATH"
        )
        return False
    
    def _install_mkcert_macos(self) -> bool:
        """Install mkcert on macOS using Homebrew."""
        if not shutil.which('brew'):
            logger.error(
                "Homebrew not found. Please install Homebrew first:\n"
                "  /bin/bash -c \"$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)\""
            )
            return False
        
        logger.info("Installing mkcert via Homebrew...")
        result = subprocess.run(
            ['brew', 'install', 'mkcert'],
            capture_output=True,
            text=True,
            timeout=300
        )
        
        if result.returncode == 0:
            logger.info("✓ mkcert installed via Homebrew")
            return True
        else:
            logger.error(f"Homebrew installation failed: {result.stderr}")
            return False
    
    def _install_mkcert_linux(self) -> bool:
        """Install mkcert on Linux - not supported, user must install manually."""
        logger.warning(
            "Automatic mkcert installation not supported on Linux.\n"
            "Please install mkcert manually:\n"
            "  Ubuntu/Debian: sudo apt install mkcert\n"
            "  Arch: sudo pacman -S mkcert\n"
            "  Or download from: https://github.com/FiloSottile/mkcert/releases"
        )
        return False
    
    def _refresh_windows_path(self):
        """Refresh PATH environment variable on Windows."""
        if platform.system() == 'Windows':
            try:
                import winreg
                # Read system PATH
                with winreg.OpenKey(winreg.HKEY_LOCAL_MACHINE, 
                                   r'SYSTEM\CurrentControlSet\Control\Session Manager\Environment') as key:
                    system_path = winreg.QueryValueEx(key, 'PATH')[0]
                
                # Read user PATH
                with winreg.OpenKey(winreg.HKEY_CURRENT_USER, r'Environment') as key:
                    user_path = winreg.QueryValueEx(key, 'PATH')[0]
                
                # Update current process PATH
                os.environ['PATH'] = f"{system_path};{user_path}"
                
            except Exception as e:
                logger.debug(f"Could not refresh Windows PATH: {e}")
    
    def _generate_self_signed(self):
        """Generate self-signed certificate using OpenSSL."""
        try:
            # Check if OpenSSL is available
            if not shutil.which('openssl'):
                raise RuntimeError(
                    "OpenSSL not found. Cannot generate certificates.\n"
                    "Please install OpenSSL or mkcert."
                )
            
            # Generate self-signed certificate
            cmd = [
                'openssl', 'req', '-x509', '-nodes',
                '-days', '365',
                '-newkey', 'rsa:2048',
                '-keyout', str(self.key_path),
                '-out', str(self.cert_path),
                '-subj', '/CN=localhost/O=R0B0 Robot/C=US',
                '-addext', 'subjectAltName=DNS:localhost,DNS:*.local,IP:127.0.0.1,IP:192.168.1.*'
            ]
            
            result = subprocess.run(
                cmd,
                capture_output=True,
                text=True,
                timeout=30,
                cwd=str(self.keys_dir)
            )
            
            if result.returncode != 0:
                raise RuntimeError(f"OpenSSL failed: {result.stderr}")
                
            logger.info(f"Generated self-signed certificate: {self.cert_path}")
            
        except Exception as e:
            raise RuntimeError(f"Failed to generate self-signed certificate: {e}")


def ensure_https_certificates(cert_path: Path, key_path: Path) -> tuple[Path, Path]:
    """
    Convenience function to ensure HTTPS certificates exist.
    
    Args:
        cert_path: Path to certificate file
        key_path: Path to private key file
        
    Returns:
        tuple: (cert_path, key_path)
    """
    manager = CertificateManager(cert_path, key_path)
    return manager.ensure_certificates()
