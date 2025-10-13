"""
Certificate management utilities for HTTPS support.
Handles automatic generation of locally-trusted certificates.
"""

import subprocess
import shutil
from pathlib import Path
from datetime import datetime
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
        if self._certificates_exist():
            logger.info("✓ SSL certificates found")
            
            # Check expiry
            days_remaining = self._check_certificate_expiry()
            if days_remaining is not None and days_remaining < 7:
                logger.warning(f"Certificate expires in {days_remaining} days. Regenerating...")
                self._generate_self_signed()
            else:
                logger.info(
                    f"Certificate valid for {days_remaining} more days"
                    if days_remaining is not None
                    else "Could not determine expiry; keeping existing certificate"
                )
        else:
            logger.info("No certificates found. Generating new certificates...")
            self._generate_self_signed()
            
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
            result = subprocess.run(
                ['openssl', 'x509', '-enddate', '-noout', '-in', str(self.cert_path)],
                capture_output=True,
                text=True,
                timeout=5
            )
            
            if result.returncode != 0:
                logger.debug(f"openssl x509 failed: {result.stderr}")
                return None
            
            # Parse output: "notAfter=Oct  3 12:00:00 2026 GMT"
            expiry_str = result.stdout.strip().split('=')[1]
            expiry_date = datetime.strptime(expiry_str, '%b %d %H:%M:%S %Y %Z')
            return (expiry_date - datetime.now()).days
            
        except Exception as exc:
            logger.debug(f"Could not check certificate expiry: {exc}")
            return None
    
    def _generate_self_signed(self):
        """Generate self-signed certificate using OpenSSL."""
        try:
            # Ensure .keys directory exists
            self.keys_dir.mkdir(parents=True, exist_ok=True)
            
            # Check if OpenSSL is available
            if not shutil.which('openssl'):
                raise RuntimeError("OpenSSL not found. Cannot generate certificates.")
            
            # Generate self-signed certificate
            cmd = [
                'openssl', 'req', '-x509', '-nodes',
                '-days', '365',
                '-newkey', 'rsa:2048',
                '-keyout', str(self.key_path),
                '-out', str(self.cert_path),
                '-subj', '/C=US/O=r0b0/CN=localhost'
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
                
            logger.info(f"✓ Generated self-signed certificate: {self.cert_path}")
            
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
