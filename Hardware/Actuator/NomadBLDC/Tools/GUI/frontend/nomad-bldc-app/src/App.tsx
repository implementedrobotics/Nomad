// React
import React, { useEffect } from 'react';

// Router
import { Route, BrowserRouter as Router, Routes, Outlet, useNavigate } from 'react-router-dom';

// Pages
import Home from './Pages/Home';
import RealtimeVisualizer from './Pages/RealtimeVisualizer';
import Settings from './Pages/Settings';

// MUI
import { Box, Paper, Container, ListItemButton, ListItemIcon, ListItemText } from '@mui/material';
import Grid from '@mui/material/Grid';

// Icons
import DashboardIcon from '@mui/icons-material/Dashboard';
import TimelineIcon from '@mui/icons-material/Timeline';
import SettingsIcon from '@mui/icons-material/Settings';

import ErrorPage from 'Pages/ErrorPage';

function NavigationMenu() {
    let navigate = useNavigate();

    const [selectedIndex, setSelectedIndex] = React.useState(0);

    return (
        <>
            <ListItemButton
                onClick={() => {
                    navigate('/');
                    setSelectedIndex(0);
                }}
                selected={selectedIndex === 0}
            >
                <ListItemIcon>
                    <DashboardIcon />
                </ListItemIcon>
                <ListItemText primary="Device Dashboard" />
            </ListItemButton>
            <ListItemButton
                onClick={() => {
                    navigate('realtime');
                    setSelectedIndex(1);
                }}
                selected={selectedIndex === 1}
            >
                <ListItemIcon>
                    <TimelineIcon />
                </ListItemIcon>
                <ListItemText primary="Realtime Telemetry" />
            </ListItemButton>
            <ListItemButton
                onClick={() => {
                    navigate('settings');
                    setSelectedIndex(2);
                }}
                selected={selectedIndex === 2}
            >
                <ListItemIcon>
                    <SettingsIcon />
                </ListItemIcon>
                <ListItemText primary="Settings" />
            </ListItemButton>
        </>
    );
}
function App() {
    return (
        <Router>
            <Box sx={{ display: 'flex', backgroundColor: '#EEEEEE', height: '100vh' }}>
                <Paper
                    elevation={4}
                    square={false}
                    sx={{
                        width: 240,
                        margin: '12px',
                    }}
                >
                    <NavigationMenu />
                </Paper>
                <Routes>
                    <Route path="/" element={<Home />} />
                    <Route path="/realtime" element={<RealtimeVisualizer />} />
                    <Route path="/settings" element={<Settings />} />
                    <Route path="*" element={<ErrorPage />} />
                </Routes>
            </Box>
        </Router>
    );
}
export default App;
